#!/usr/bin/env python3
"""3. adım: servo açısı <-> model açısı kalibrasyonu (Pi'de, kol bağlıyken çalışır).

Fikir:
  - Kanal 1 (komut o) ve kanal 2 (komut d) servolarından biri 1-5 üst kolunu, öbürü 1-V tahrik
    kolunu döndürür. Hangisinin hangisi olduğu kablolamaya bağlı (07.11: kanal 2 üst kol,
    23.11 servo değişimi sonrası: kanal 1 üst kol); hesapla ölçümlerden kendisi bulur.
    İkisi de tabana sabit olduğundan  model = a * servo + b  (a ≈ +1 ya da -1) beklenir.
  - Açı girerken sonuna g yazılırsa grad kabul edilir (85g -> 76,5 derece).
  - Tahrik kolu küçük ve görünmesi zor; bu yüzden onu değil, uzun ön kolu (5 -> 7) ölçeriz.
    Üst kol açısı = model omuz açısıdır. Ön kol açısı + omuz açısından model ile tahrik bulunur.
  - Her duruşta ayrıca tabla eğimi (isteğe bağlı) ölçülür -> modelin "tabla yatay" varsayımı denetlenir.
  - Kıskaç: servo açısı -> parmak açıklığı (mm) noktaları, ara değerler doğrusal ara değerlemeyle.

Açı yönü (telefonda eğim ölçer, ya da açıölçer): kolu YANDAN, ileri yön SAĞDA olacak şekilde bak.
  Yatay-ileri = 0, dik yukarı = 90, geriye yatık > 90, ileri-aşağı eğik = eksi.
  üst kol : omuz ekseninden (1) dirsek eksenine (5) doğru
  ön kol  : dirsek ekseninden (5) kıskaç montaj eksenine (7) doğru
  tabla   : kıskaç tablasının ön ucu yukarıdaysa +, aşağıdaysa -

Komutlar:
  b 90 / o 100 / d 80 / k 30   taban / omuz / dirsek / kıskaç servosunu o açıya götür
  o+ / o- / d+ / d-            1 derece oynat (o++ / o-- = 5 derece)
  olc                          bu duruşta ölçüm gir (üst kol, ön kol, tabla)
  kol                          kıskaç ölçümü gir (açıklık mm)
  liste / sil N                kayıtları göster / N. kaydı sil
  hesapla                      doğru uydur, hataları göster, kalibrasyon.json'a yaz
  q                            çık (kayıtlar her ölçümde zaten dosyaya yazılır)
"""
import json
import math
import os
import sys

import kinematik as k

DOSYA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kalibrasyon.json")
SURE_MS = 1500  # yavaş hareket


def onkol_acisi(omuz, tahrik):
    p = k.ileri(omuz, tahrik)
    if p is None:
        return None
    return math.degrees(math.atan2(p[7][1] - p[5][1], p[7][0] - p[5][0]))


def tahrik_bul(omuz, onkol_olculen, aralik=(90.0, 260.0), adim=0.1):
    """Ölçülen ön kol açısını veren model tahrik açısı (tarama + en yakın). (tahrik, hata) ya da None."""
    en_iyi = None
    t = aralik[0]
    while t <= aralik[1]:
        f = onkol_acisi(omuz, t)
        if f is not None:
            h = abs(f - onkol_olculen)
            if en_iyi is None or h < en_iyi[1]:
                en_iyi = (t, h)
        t += adim
    return en_iyi


def dogru_uydur(xs, ys):
    """En küçük kareler y = a*x + b. (a, b, artıklar)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    return a, b, [y - (a * x + b) for x, y in zip(xs, ys)]


def yukle():
    if os.path.exists(DOSYA):
        with open(DOSYA, encoding="utf-8") as f:
            return json.load(f)
    return {"duruslar": [], "kiskac": [], "sonuc": None}


def kaydet(veri):
    with open(DOSYA, "w", encoding="utf-8") as f:
        json.dump(veri, f, ensure_ascii=False, indent=2)


def sayi_sor(soru, bos_olur=False):
    while True:
        s = input(soru).strip().replace(",", ".").lower()
        if not s and bos_olur:
            return None
        try:
            if s.endswith("g"):          # açıölçer grad gösteriyor: 100 grad = 90 derece
                return float(s[:-1]) * 0.9
            return float(s)
        except ValueError:
            print("  sayı gir" + (" (ya da boş bırak)" if bos_olur else ""))


def hesapla(veri):
    d = veri["duruslar"]
    sonuc = {}
    satirlar = []
    for r in d:
        r["model_omuz"] = r["ust_kol"]
        tb = tahrik_bul(r["ust_kol"], r["on_kol"])
        r["model_tahrik"] = None if tb is None else round(tb[0], 2)
        r["tahrik_uyum_hatasi"] = None if tb is None else round(tb[1], 2)
        satirlar.append(r)

    if len(d) < 2:
        print("en az 2 duruş gerekli (öneri: 5-6, omuz ve dirseği geniş aralıkta değiştir)")
        return
    print("\nduruş  servo(o,d)   ölçülen(üst,ön)   model(omuz,tahrik)  ön kol uyum")
    for i, r in enumerate(satirlar):
        print(f"{i:4d}  {r['servo'][1]:4d},{r['servo'][2]:4d}   {r['ust_kol']:6.1f},{r['on_kol']:6.1f}"
              f"     {r['model_omuz']:6.1f},{r['model_tahrik'] or float('nan'):6.1f}"
              f"      {r['tahrik_uyum_hatasi'] if r['tahrik_uyum_hatasi'] is not None else '-'}")
        if r["tahrik_uyum_hatasi"] is None or r["tahrik_uyum_hatasi"] > 1.0:
            print("       ! bu ön kol açısı modelde bu omuz açısıyla elde edilemiyor (ölçüm/işaret hatası?)")

    # Hangi kanalın hangi kolu sürdüğü servo değişiminde değişebiliyor (07.11: kanal 2 üst kol,
    # 23.11: kanal 1 üst kol). Bu yüzden iki kanal da denenir, ölçüme en iyi uyan seçilir.
    for ad, anahtar in (("omuz", "model_omuz"), ("tahrik", "model_tahrik")):
        en_iyi = None
        for kanal in (1, 2):
            ciftler = [(r["servo"][kanal], r[anahtar]) for r in satirlar if r[anahtar] is not None]
            u = dogru_uydur([c[0] for c in ciftler], [c[1] for c in ciftler]) if len(ciftler) >= 2 else None
            if u is None or abs(u[0]) < 0.3:      # eğim ~0: bu kanal bu kolu oynatmıyor
                continue
            en_kotu = max(abs(x) for x in u[2])
            if en_iyi is None or en_kotu < en_iyi[0]:
                en_iyi = (en_kotu, kanal, u)
        if en_iyi is None:
            print(f"\n{ad}: uydurulamadı (bu kolu oynatan servo değişmedi mi?)")
            continue
        en_kotu, kanal, (a, b, art) = en_iyi
        print(f"\n{ad}: model = {a:+.3f} * servo{kanal} {b:+.1f}   (en büyük artık {en_kotu:.1f}°)")
        if abs(abs(a) - 1) > 0.1:
            print(f"  ! eğim {a:.2f}; 1'e yakın olması beklenir -> ölçümleri kontrol et")
        if en_kotu > 2.5:
            print("  ! artık büyük -> bir ölçüm hatalı olabilir, 'liste' ile bak")
        smin, smax = k_sinir(kanal)
        uclar = sorted((a * smin + b, a * smax + b))
        print(f"  servo {smin}..{smax} -> model {uclar[0]:.1f}..{uclar[1]:.1f}")
        sonuc[ad] = {"kanal": kanal, "a": a, "b": b, "en_buyuk_artik": en_kotu,
                     "model_aralik": uclar}

    egimler = [(r["tabla"], k.tabla_egimi_deg(r["model_omuz"], r["model_tahrik"]))
               for r in satirlar if r.get("tabla") is not None and r["model_tahrik"] is not None]
    egimler = [(o, m) for o, m in egimler if m is not None]
    if egimler:
        farklar = [o - m for o, m in egimler]
        ort = sum(farklar) / len(farklar)
        yayilma = max(farklar) - min(farklar)
        print(f"\ntabla: ölçülen - model ortalama {ort:+.1f}°, yayılma {yayilma:.1f}° ({len(egimler)} duruş)")
        for o, m in egimler:
            print(f"  ölçülen {o:+6.1f}  model {m:+6.1f}")
        if yayilma > 4:
            print("  ! fark duruşa göre değişiyor -> sabit kayma değil, model geometrisini incelemek gerek")
        sonuc["tabla_kaymasi_deg"] = ort
        sonuc["tabla_yayilma_deg"] = yayilma

    kis = sorted(veri["kiskac"], key=lambda r: r["servo"])
    if len(kis) >= 2:
        print("\nkıskaç servo -> açıklık:")
        for r in kis:
            print(f"  {r['servo']:4d}° -> {r['mm']:5.1f} mm")
        sonuc["kiskac_tablo"] = [[r["servo"], r["mm"]] for r in kis]

    veri["sonuc"] = sonuc
    kaydet(veri)
    print(f"\nyazıldı: {DOSYA}")


def k_sinir(kanal):
    try:
        from kol_seri import SERVO_MIN, SERVO_MAX
    except ImportError:   # --hesapla PC'de, kol_seri/pyserial olmadan
        SERVO_MIN, SERVO_MAX = (5, 50, 55, 0), (175, 145, 110, 135)
    return SERVO_MIN[kanal], SERVO_MAX[kanal]


def main():
    from kol_seri import Kol, KolHatasi, SERVO_MIN, SERVO_MAX

    veri = yukle()
    print(__doc__.split("Komutlar:")[1])
    print(f"kayıtlı: {len(veri['duruslar'])} duruş, {len(veri['kiskac'])} kıskaç ölçümü")
    kol = Kol()
    if not kol.ping():
        sys.exit("STM32 cevap vermiyor")
    aci, _ = kol.konum()
    harf = {"b": 0, "o": 1, "d": 2, "k": 3}

    while True:
        try:
            s = input(f"[{aci[0]},{aci[1]},{aci[2]},{aci[3]}] > ").strip().lower()
        except EOFError:
            break
        if not s:
            continue
        try:
            if s == "q":
                break
            elif s == "hesapla":
                hesapla(veri)
            elif s == "liste":
                for i, r in enumerate(veri["duruslar"]):
                    print(i, r)
                for i, r in enumerate(veri["kiskac"]):
                    print(f"k{i}", r)
            elif s.startswith("sil"):
                p = s.split()[1]
                if p.startswith("k"):
                    print("silindi:", veri["kiskac"].pop(int(p[1:])))
                else:
                    print("silindi:", veri["duruslar"].pop(int(p)))
                kaydet(veri)
            elif s == "olc":
                print("  Kol durdu mu? Açıları yandan ölç (ileri sağda).")
                ust = sayi_sor("  üst kol açısı (1->5): ")
                on = sayi_sor("  ön kol açısı (5->7): ")
                tabla = sayi_sor("  tabla eğimi (boş = ölçmedim): ", bos_olur=True)
                tb = tahrik_bul(ust, on)
                if tb is None or tb[1] > 1.0:
                    print("  ! bu iki açı modelde birlikte olmuyor; yön/işaret doğru mu? Yine de kaydedildi.")
                else:
                    print(f"  model: omuz {ust:.1f}, tahrik {tb[0]:.1f}")
                veri["duruslar"].append({"servo": list(aci), "ust_kol": ust, "on_kol": on,
                                         "tabla": tabla})
                kaydet(veri)
            elif s == "kol":
                mm = sayi_sor(f"  kıskaç {aci[3]}° iken parmak iç yüzleri arası (mm): ")
                veri["kiskac"].append({"servo": aci[3], "mm": mm})
                kaydet(veri)
            elif s[0] in harf:
                kanal = harf[s[0]]
                yeni = list(aci)
                kalan = s[1:].strip()
                if kalan in ("+", "-", "++", "--"):
                    yeni[kanal] += (1 if "+" in kalan else -1) * (5 if len(kalan) == 2 else 1)
                else:
                    yeni[kanal] = int(float(kalan))
                if not SERVO_MIN[kanal] <= yeni[kanal] <= SERVO_MAX[kanal]:
                    print(f"  sınır dışı ({SERVO_MIN[kanal]}-{SERVO_MAX[kanal]})")
                    continue
                kol.git(*yeni, sure_ms=SURE_MS)
                aci = yeni
            else:
                print("  bilinmeyen komut")
        except (KolHatasi, ValueError, IndexError) as e:
            print("  !", e)
        except KeyboardInterrupt:
            kol.dur()
            aci, _ = kol.konum()
            print("\n  durduruldu")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--hesapla":
        hesapla(yukle())   # kol bağlı olmadan, kayıtlı ölçümlerden
    else:
        main()
