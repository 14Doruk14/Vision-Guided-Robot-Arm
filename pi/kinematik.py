#!/usr/bin/env python3
"""Robot kol ileri / ters kinematiği (model açılarıyla).

Koordinatlar mm. Yan düzlemde x = ileri (taban açısı 0 iken), z = zeminden yükseklik.
Nokta 1 (omuz ekseni) (0, 61)'de. Nokta 7 = ön alt montaj ekseni = kıskaçtaki K1.
Tutma noktası M = parmak uçlarındaki iç tutma yüzlerinin ortası (kıskaç sürüm 5, 30.10.2026).
Kıskaç tablası 6-7 çubuğuna bağlı döner ve mekanizma paralelkenar olmadığı için
duruşa göre eğilir (model çalışma alanında yaklaşık -42..+59 derece).

Açılar MODEL açısıdır, servo açısı değildir:
    omuz   = 1-5 çubuğunun yatayla açısı (65..145 derece taranır)
    tahrik = 1-V kısa tahrik kolunun yatayla açısı (115..235 derece taranır)
Servo açısına çevirme 3. adımdaki kalibrasyonla yapılacak.

Kullanım:
    python kinematik.py 120 40             # M noktası (x, z) için IK, kıskaç kapalı (4 mm)
    python kinematik.py 120 40 --aciklik 30  # M noktası, parmaklar 30 mm açıkken
    python kinematik.py 100 50 40          # (x, y, z) için IK, taban açısıyla
    python kinematik.py 80 110 --nokta7    # eski davranış: nokta 7 için IK
    python kinematik.py --tablo tablo.csv  # IK tablosunu dosyaya yaz
"""
import argparse
import math
from collections import deque

# --- Ölçüler (arsiv/03_olculer/son-olculer-ve-proje-kaydi.json, model_lengths_mm) ---
L = {
    "1-2": 40, "1-5": 77, "2-3": 85, "3-4": 47, "3-5": 40, "4-5": 20,
    "4-6": 80, "6-7": 23, "1-V": 40, "V-R": 80, "R-5": 35, "5-7": 80,
}
P1 = (0.0, 61.0)
_REF = math.hypot(36.1, 17.23)                 # sabit 2 noktasının yönü (referans montaj)
P2 = (-36.1 * 40 / _REF, 61 + 17.23 * 40 / _REF)

OMUZ_ARALIK = (65.0, 145.0)
TAHRIK_ARALIK = (115.0, 235.0)

# --- Kıskaç (sürüm 5 paketi, kaynak-kod/kiskac-kinematigi.js) ---
# Kullanıcı ölçüsü: kapalıyken (4 mm) ve tabla yatayken K1 -> M: 47,5 mm ileri, 37 mm aşağı.
# Açıklık 4..79 mm. Parmak eksen aralığı 18,1 mm ve ölçek 0,84 MODEL VARSAYIMI (ölçülmedi).
KISKAC_ILERI = 47.5
KISKAC_DUSEY = -37.0
ACIKLIK_MIN, ACIKLIK_MAX = 4.0, 79.0
_PARMAK_EKSEN_ARALIGI = 18.1
_PARMAK_OLCEK = 0.84
# Tabla "yatay" kabul edilen 7->6 yönü (referans montajdan; fiziksel olarak doğrulanmadı).
_TABLA_YATAY = math.atan2(17.23, 10.16)


def _polar(r, aci_deg):
    a = math.radians(aci_deg)
    return (r * math.cos(a), r * math.sin(a))


def _cember(a, r, b, s):
    """a merkezli r ve b merkezli s yarıçaplı çemberlerin a->b yönünün solundaki kesişimi.
    kinematik.js'teki circle() ile aynı; kesişim yoksa None."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(vx, vy)
    if d < 1e-8 or d > r + s + 1e-7 or d < abs(r - s) - 1e-7:
        return None
    ux, uy = vx / d, vy / d
    t = (r * r - s * s + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r * r - t * t))
    cx, cy = a[0] + ux * t, a[1] + uy * t
    return (cx - h * uy, cy + h * ux)


def ileri(omuz_deg, tahrik_deg):
    """Tüm eklem noktalarını döndürür; mekanizma kapanmıyorsa None."""
    p = {1: P1, 2: P2}
    o = _polar(L["1-5"], omuz_deg)
    p[5] = (P1[0] + o[0], P1[1] + o[1])
    p[3] = _cember(p[2], L["2-3"], p[5], L["3-5"])
    if p[3] is None:
        return None
    p[4] = _cember(p[3], L["3-4"], p[5], L["4-5"])
    if p[4] is None:
        return None
    v = _polar(L["1-V"], tahrik_deg)
    p["V"] = (P1[0] + v[0], P1[1] + v[1])
    p["R"] = _cember(p["V"], L["V-R"], p[5], L["R-5"])
    if p["R"] is None:
        return None
    dx, dy = p[5][0] - p["R"][0], p[5][1] - p["R"][1]
    d = math.hypot(dx, dy)
    p[7] = (p[5][0] + dx * L["5-7"] / d, p[5][1] + dy * L["5-7"] / d)
    p[6] = _cember(p[4], L["4-6"], p[7], L["6-7"])
    if p[6] is None:
        return None
    return p


def uc_noktasi(omuz_deg, tahrik_deg):
    p = ileri(omuz_deg, tahrik_deg)
    return None if p is None else p[7]


def tabla_acisi(p):
    """Kıskaç tablasının yataydan eğimi (radyan, + = ön uç yukarı). p = ileri() sonucu."""
    return math.atan2(p[6][1] - p[7][1], p[6][0] - p[7][0]) - _TABLA_YATAY


def kiskac_ileri(aciklik_mm):
    """Tabla çerçevesinde K1 -> M ileri uzaklığı (mm). Parmaklar açıldıkça uçlar geriye
    döndüğü için M geri gelir: 4 mm -> 47,5; 30 mm -> 47,7; 79 mm -> 32,1.
    kiskac-kinematigi.js'teki iki parmağın temas noktalarının ortalaması."""
    g = min(max(aciklik_mm, ACIKLIK_MIN), ACIKLIK_MAX)
    yarim = _PARMAK_EKSEN_ARALIGI / 2
    toplam = 0.0
    for isaret, eksen_y, temas_y in ((1, 10.53, 1.6), (-1, 3.63, 12.63)):
        dx = (47.5 - 4.5) * _PARMAK_OLCEK
        dy = isaret * (temas_y - eksen_y) * _PARMAK_OLCEK
        r = math.hypot(dx, dy)
        ilk = ACIKLIK_MIN / 2 - yarim
        eksen_ileri = KISKAC_ILERI - math.sqrt(r * r - ilk * ilk)
        don = math.asin((g / 2 - yarim) / r) - math.atan2(dy, dx)
        toplam += eksen_ileri + dx * math.cos(don) - dy * math.sin(don)
    return toplam / 2


def _tutma(p7, aci, m_ileri):
    c, s = math.cos(aci), math.sin(aci)
    return (p7[0] + c * m_ileri - s * KISKAC_DUSEY, p7[1] + s * m_ileri + c * KISKAC_DUSEY)


def tutma_noktasi(omuz_deg, tahrik_deg, aciklik_mm=ACIKLIK_MIN):
    """Kıskaç tutma merkezi M (x, z); mekanizma kapanmıyorsa None."""
    p = ileri(omuz_deg, tahrik_deg)
    if p is None:
        return None
    return _tutma(p[7], tabla_acisi(p), kiskac_ileri(aciklik_mm))


def tabla_egimi_deg(omuz_deg, tahrik_deg):
    p = ileri(omuz_deg, tahrik_deg)
    return None if p is None else math.degrees(tabla_acisi(p))


def _uc_fonksiyonu(aciklik_mm):
    """aciklik_mm None -> nokta 7, sayı -> o açıklıktaki M noktası."""
    if aciklik_mm is None:
        return uc_noktasi
    return lambda o, t: tutma_noktasi(o, t, aciklik_mm)


class IKTablosu:
    """Açı ızgarasını tarar; her geçerli hücre için nokta 7'yi, tabla açısını ve
    geçersiz bölgeye uzaklığı (pay, derece) tutar. M noktası açıklığa göre
    çözüm sırasında bunlardan hesaplanır."""

    def __init__(self, adim=0.5, omuz_aralik=OMUZ_ARALIK, tahrik_aralik=TAHRIK_ARALIK):
        self.adim = adim
        self.o0, self.t0 = omuz_aralik[0], tahrik_aralik[0]
        self.no = int(round((omuz_aralik[1] - omuz_aralik[0]) / adim)) + 1
        self.nt = int(round((tahrik_aralik[1] - tahrik_aralik[0]) / adim)) + 1

        self.uc = [[None] * self.nt for _ in range(self.no)]
        self.tabla = [[None] * self.nt for _ in range(self.no)]
        for i in range(self.no):
            for j in range(self.nt):
                p = ileri(self.aci_o(i), self.aci_t(j))
                if p is not None:
                    self.uc[i][j] = p[7]
                    self.tabla[i][j] = tabla_acisi(p)
        self._pay_hesapla()
        self.gecerli = [(i, j) for i in range(self.no) for j in range(self.nt)
                        if self.uc[i][j] is not None]

    def aci_o(self, i):
        return self.o0 + i * self.adim

    def aci_t(self, j):
        return self.t0 + j * self.adim

    def _pay_hesapla(self):
        """Geçersiz hücrelerden başlayan çok kaynaklı BFS (8 komşu).
        pay = en yakın geçersiz hücreye adım sayısı x adim (derece).
        Tarama aralığının kenarı geçersiz sayılmaz: aralık dışı bilinmiyor."""
        sonsuz = 10 ** 9
        dist = [[sonsuz] * self.nt for _ in range(self.no)]
        q = deque()
        for i in range(self.no):
            for j in range(self.nt):
                if self.uc[i][j] is None:
                    dist[i][j] = 0
                    q.append((i, j))
        while q:
            i, j = q.popleft()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < self.no and 0 <= b < self.nt and dist[a][b] > dist[i][j] + 1:
                        dist[a][b] = dist[i][j] + 1
                        q.append((a, b))
        self.pay = [[(d * self.adim if d < sonsuz else math.inf) for d in satir] for satir in dist]

    def pay_deg(self, omuz, tahrik):
        """Açı çiftine en yakın ızgara hücresinin payı; aralık dışıysa 0."""
        i = round((omuz - self.o0) / self.adim)
        j = round((tahrik - self.t0) / self.adim)
        if not (0 <= i < self.no and 0 <= j < self.nt):
            return 0.0
        return self.pay[i][j]

    def hucre_ucu(self, i, j, aciklik_mm=None):
        """Hücrenin uç noktası: aciklik_mm None -> nokta 7, sayı -> M."""
        if aciklik_mm is None:
            return self.uc[i][j]
        return _tutma(self.uc[i][j], self.tabla[i][j], kiskac_ileri(aciklik_mm))

    def coz(self, x, z, aciklik_mm=None, min_pay=3.0, tolerans_mm=0.1):
        """(x, z) için (omuz, tahrik) model açıları. aciklik_mm verilirse hedef,
        parmaklar o açıklıktayken M tutma noktasıdır; verilmezse nokta 7.
        Ulaşılamıyorsa ya da yalnız geçersiz bölgeye yakın çözüm varsa None."""
        m_ileri = None if aciklik_mm is None else kiskac_ileri(aciklik_mm)
        en_iyi, en_iyi_d2 = None, math.inf
        for i, j in self.gecerli:
            if self.pay[i][j] < min_pay:
                continue
            if m_ileri is None:
                ux, uz = self.uc[i][j]
            else:
                ux, uz = _tutma(self.uc[i][j], self.tabla[i][j], m_ileri)
            d2 = (ux - x) ** 2 + (uz - z) ** 2
            if d2 < en_iyi_d2:
                en_iyi, en_iyi_d2 = (i, j), d2
        if en_iyi is None:
            return None
        a = [self.aci_o(en_iyi[0]), self.aci_t(en_iyi[1])]
        a = _newton(_uc_fonksiyonu(aciklik_mm), a, x, z, tolerans_mm)
        if a is None:
            return None
        if not (self.o0 <= a[0] <= self.aci_o(self.no - 1)
                and self.t0 <= a[1] <= self.aci_t(self.nt - 1)):
            return None
        if self.pay_deg(*a) < min_pay:
            return None
        return a[0], a[1]

    def coz_3b(self, x, y, z, **kw):
        """(x, y, z) için (taban, omuz, tahrik). Taban 0 derece = +x yönü.
        Taban dönüş ekseni nokta 1'in tam altında kabul edilir (ofset 0, ölçülmedi)."""
        taban = math.degrees(math.atan2(y, x))
        r = math.hypot(x, y)
        sonuc = self.coz(r, z, **kw)
        if sonuc is None:
            return None
        return (taban,) + sonuc

    def csv_yaz(self, yol):
        with open(yol, "w", encoding="utf-8") as f:
            f.write("omuz_deg,tahrik_deg,x7_mm,z7_mm,xM_mm,zM_mm,tabla_deg,pay_deg\n")
            for i, j in self.gecerli:
                ux, uz = self.uc[i][j]
                mx, mz = self.hucre_ucu(i, j, ACIKLIK_MIN)
                f.write(f"{self.aci_o(i):.1f},{self.aci_t(j):.1f},{ux:.3f},{uz:.3f},"
                        f"{mx:.3f},{mz:.3f},{math.degrees(self.tabla[i][j]):.2f},"
                        f"{self.pay[i][j]:.1f}\n")


def _newton(uc, a, x, z, tolerans_mm, en_fazla=30):
    """Sayısal Jacobian'lı Newton; adım mekanizmayı geçersiz bölgeye atarsa yarıya iner.
    uc(omuz, tahrik) -> (x, z) ya da None."""
    h = 1e-4
    for _ in range(en_fazla):
        p = uc(*a)
        if p is None:
            return None
        ex, ez = x - p[0], z - p[1]
        if math.hypot(ex, ez) < tolerans_mm:
            return a
        po, pt = uc(a[0] + h, a[1]), uc(a[0], a[1] + h)
        if po is None or pt is None:
            return None
        j11, j21 = (po[0] - p[0]) / h, (po[1] - p[1]) / h
        j12, j22 = (pt[0] - p[0]) / h, (pt[1] - p[1]) / h
        det = j11 * j22 - j12 * j21
        if abs(det) < 1e-9:
            return None
        da = (j22 * ex - j12 * ez) / det
        dt = (-j21 * ex + j11 * ez) / det
        k = 1.0
        while k > 1e-3:
            yeni = [a[0] + k * da, a[1] + k * dt]
            if uc(*yeni) is not None:
                break
            k /= 2
        else:
            return None
        a = yeni
    p = uc(*a)
    if p is not None and math.hypot(x - p[0], z - p[1]) < tolerans_mm:
        return a
    return None


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("hedef", nargs="*", type=float, help="x z  ya da  x y z (mm)")
    ap.add_argument("--tablo", help="IK tablosunu CSV olarak yaz")
    ap.add_argument("--min-pay", type=float, default=3.0, help="geçersiz bölgeye en az uzaklık (derece)")
    ap.add_argument("--aciklik", type=float, default=ACIKLIK_MIN,
                    help="M hesaplanırken parmak açıklığı, mm (4..79, varsayılan 4 = kapalı)")
    ap.add_argument("--nokta7", action="store_true", help="hedef M değil nokta 7")
    arg = ap.parse_args()

    tablo = IKTablosu()
    print(f"tablo: {len(tablo.gecerli)} geçerli / {tablo.no * tablo.nt} hücre")
    if arg.tablo:
        tablo.csv_yaz(arg.tablo)
        print("yazıldı:", arg.tablo)
    aciklik = None if arg.nokta7 else arg.aciklik
    print("hedef:", "nokta 7" if aciklik is None else f"M (açıklık {aciklik:g} mm)")
    if len(arg.hedef) == 2:
        sonuc = tablo.coz(*arg.hedef, aciklik_mm=aciklik, min_pay=arg.min_pay)
        print("omuz, tahrik =", sonuc)
    elif len(arg.hedef) == 3:
        sonuc = tablo.coz_3b(*arg.hedef, aciklik_mm=aciklik, min_pay=arg.min_pay)
        print("taban, omuz, tahrik =", sonuc)
        sonuc = sonuc and sonuc[1:]
    elif arg.hedef:
        ap.error("hedef için 2 ya da 3 sayı ver")
    if arg.hedef and sonuc:
        print(f"tabla eğimi = {tabla_egimi_deg(*sonuc):.1f} derece")
