#!/usr/bin/env python3
"""kinematik.py testleri:  python test_kinematik.py

1. Python ileri kinematiği arsivdeki kinematik.js ile aynı mı? (node varsa)
2. Doğrulama kaydındaki sayılar tutuyor mu? (2.501 çiftte 1.714 geçerli, ilk 7 noktası)
3. Bütün eklem mesafeleri ölçülerle aynı mı?
4. IK gidiş-dönüş: FK(IK(hedef)) == hedef, ve IK hiçbir zaman geçersiz bölgeye yakın çözüm vermiyor.
"""
import json
import math
import random
import shutil
import subprocess
from pathlib import Path

import kinematik as k

KOK = Path(__file__).resolve().parent.parent
JS = KOK / "arsiv" / "05_kaynak_kod" / "kinematik.js"
JS_KISKAC = KOK / "arsiv" / "kiskac_v5" / "kaynak-kod" / "kiskac-kinematigi.js"
CIFTLER = [(1, 2, 40), (1, 5, 77), (2, 3, 85), (3, 4, 47), (3, 5, 40), (4, 5, 20), (4, 6, 80),
           (6, 7, 23), (5, 7, 80), (1, "V", 40), ("V", "R", 80), ("R", 5, 35), ("R", 7, 115)]


def tarama():
    return [(a, b) for a in range(65, 146, 2) for b in range(115, 236, 2)]


def test_js_ile_ayni():
    node = shutil.which("node")
    if not node:
        print("ATLANDI: node yok, JS karşılaştırması yapılmadı")
        return
    betik = JS.read_text(encoding="utf-8") + """
const out=[];for(let a=65;a<=145;a+=2)for(let b=115;b<=235;b+=2){const p=robotPose(a,b);out.push(p&&p[7]);}
console.log(JSON.stringify(out));"""
    js = json.loads(subprocess.run([node, "-e", betik], capture_output=True, text=True, check=True).stdout)
    en_kotu = 0.0
    for (a, b), beklenen in zip(tarama(), js):
        p = k.uc_noktasi(a, b)
        assert (p is None) == (beklenen is None), (a, b)
        if p:
            en_kotu = max(en_kotu, math.dist(p, beklenen))
    assert en_kotu < 1e-9, en_kotu
    print(f"TAMAM js ile aynı ({len(js)} çift, en büyük fark {en_kotu:.1e} mm)")


def test_kiskac_js_ile_ayni():
    node = shutil.which("node")
    if not node:
        print("ATLANDI: node yok, kıskaç JS karşılaştırması yapılmadı")
        return
    betik = JS.read_text(encoding="utf-8") + JS_KISKAC.read_text(encoding="utf-8") + """
const out=[];for(let a=65;a<=145;a+=4)for(let b=115;b<=235;b+=4)for(const y of [0,20,35,60,100]){
const p=robotPose(a,b);out.push(p&&robotGripper(p,0,y).worldPoints.M);}
console.log(JSON.stringify(out));"""
    js = json.loads(subprocess.run([node, "-e", betik], capture_output=True, text=True, check=True).stdout)
    en_kotu, n = 0.0, 0
    for a in range(65, 146, 4):
        for b in range(115, 236, 4):
            for y in (0, 20, 35, 60, 100):
                beklenen = js[n]
                n += 1
                m = k.tutma_noktasi(a, b, 4 + 0.75 * y)     # JS: açıklık yüzdesi
                assert (m is None) == (beklenen is None), (a, b)
                if m:
                    assert abs(beklenen[2]) < 1e-9              # taban 0: yanal sapma yok
                    en_kotu = max(en_kotu, math.dist(m, beklenen[:2]))
    assert en_kotu < 1e-9, en_kotu
    print(f"TAMAM kıskaç M noktası js ile aynı ({n} durum, en büyük fark {en_kotu:.1e} mm)")


def test_kiskac_olculeri():
    # Tabla yatay olduğu duruşta K1 -> M = (47,5 ileri, 37 aşağı) olmalı
    assert abs(k.kiskac_ileri(4) - 47.5) < 1e-9
    assert abs(k.kiskac_ileri(79) - 32.11968484505708) < 1e-9   # v5 JSON open_model_local_points
    p = k.ileri(125, 180)
    m = k.tutma_noktasi(125, 180)
    a = k.tabla_acisi(p)
    yerel = ((m[0] - p[7][0]) * math.cos(a) + (m[1] - p[7][1]) * math.sin(a),
             -(m[0] - p[7][0]) * math.sin(a) + (m[1] - p[7][1]) * math.cos(a))
    assert math.dist(yerel, (47.5, -37)) < 1e-9, yerel
    print(f"TAMAM kıskaç ölçüleri; ilk duruşta tabla eğimi {math.degrees(a):.1f} derece, "
          f"M = ({m[0]:.1f}, {m[1]:.1f})")


def test_dogrulama_kaydi():
    gecerli = sum(k.ileri(a, b) is not None for a, b in tarama())
    assert gecerli == 1714, gecerli
    p7 = k.uc_noktasi(125, 180)
    assert math.dist(p7, (34.17867389423205, 107.88185892346249)) < 1e-9, p7
    print(f"TAMAM doğrulama kaydı: {gecerli} geçerli, ilk 7 noktası ({p7[0]:.2f}, {p7[1]:.2f})")


def test_mesafeler():
    en_kotu = 0.0
    for a, b in tarama():
        p = k.ileri(a, b)
        if p:
            for i, j, d in CIFTLER:
                en_kotu = max(en_kotu, abs(math.dist(p[i], p[j]) - d))
    assert en_kotu < 1e-7, en_kotu
    print(f"TAMAM 13 mesafe, en büyük hata {en_kotu:.1e} mm")


def test_ik(tablo):
    rnd = random.Random(1)
    adaylar = [c for c in tablo.gecerli if tablo.pay[c[0]][c[1]] >= 6]
    basarili = 0
    for _ in range(200):
        i, j = rnd.choice(adaylar)
        # ızgara dışı rastgele açı -> hedef nokta
        a = min(max(tablo.aci_o(i) + rnd.uniform(-0.25, 0.25), k.OMUZ_ARALIK[0]), k.OMUZ_ARALIK[1])
        b = min(max(tablo.aci_t(j) + rnd.uniform(-0.25, 0.25), k.TAHRIK_ARALIK[0]), k.TAHRIK_ARALIK[1])
        hedef = k.uc_noktasi(a, b)
        sonuc = tablo.coz(*hedef)
        if sonuc is None:
            continue
        basarili += 1
        assert math.dist(k.uc_noktasi(*sonuc), hedef) < 0.1
        assert tablo.pay_deg(*sonuc) >= 3.0
    assert basarili == 200, basarili
    assert tablo.coz(500, 0) is None              # erişim dışı
    sonuc = tablo.coz_3b(0, 50, 100)              # tam sola: taban 90 derece
    assert sonuc and abs(sonuc[0] - 90) < 1e-9, sonuc
    assert math.dist(k.uc_noktasi(*sonuc[1:]), (50, 100)) < 0.1
    print(f"TAMAM IK gidiş-dönüş {basarili}/200, coz_3b(0,50,100) = "
          f"{tuple(round(v, 2) for v in sonuc)}")


def test_ik_tutma(tablo):
    rnd = random.Random(2)
    adaylar = [c for c in tablo.gecerli if tablo.pay[c[0]][c[1]] >= 6]
    basarili = 0
    for _ in range(200):
        i, j = rnd.choice(adaylar)
        aciklik = rnd.uniform(4, 79)
        a = min(max(tablo.aci_o(i) + rnd.uniform(-0.25, 0.25), k.OMUZ_ARALIK[0]), k.OMUZ_ARALIK[1])
        b = min(max(tablo.aci_t(j) + rnd.uniform(-0.25, 0.25), k.TAHRIK_ARALIK[0]), k.TAHRIK_ARALIK[1])
        hedef = k.tutma_noktasi(a, b, aciklik)
        sonuc = tablo.coz(*hedef, aciklik_mm=aciklik)
        if sonuc is None:
            continue
        basarili += 1
        assert math.dist(k.tutma_noktasi(*sonuc, aciklik), hedef) < 0.1
        assert tablo.pay_deg(*sonuc) >= 3.0
    assert basarili == 200, basarili
    print(f"TAMAM M noktası IK gidiş-dönüş {basarili}/200")


def ozet(tablo, aciklik_mm=None):
    ad = "nokta 7" if aciklik_mm is None else f"M, açıklık {aciklik_mm:g} mm"
    hucreler = [(i, j) for i, j in tablo.gecerli if tablo.pay[i][j] >= 3]
    noktalar = [tablo.hucre_ucu(i, j, aciklik_mm) for i, j in hucreler]
    xs, zs = [p[0] for p in noktalar], [p[1] for p in noktalar]
    print(f"çalışma alanı, {ad} (pay >= 3 derece): x {min(xs):.0f}..{max(xs):.0f} mm, "
          f"z {min(zs):.0f}..{max(zs):.0f} mm, {len(noktalar)} hücre")
    for z in (0, 20, 40, 60):
        icinde = [(p[0], math.degrees(tablo.tabla[c[0]][c[1]]))
                  for c, p in zip(hucreler, noktalar) if abs(p[1] - z) < 2]
        if icinde:
            e = [v[1] for v in icinde]
            print(f"  z = {z:>2} mm: x {min(v[0] for v in icinde):.0f}..{max(v[0] for v in icinde):.0f} mm,"
                  f" tabla eğimi {min(e):.0f}..{max(e):.0f} derece")
        else:
            print(f"  z = {z:>2} mm: ulaşılamıyor")


if __name__ == "__main__":
    test_js_ile_ayni()
    test_kiskac_js_ile_ayni()
    test_dogrulama_kaydi()
    test_mesafeler()
    test_kiskac_olculeri()
    t = k.IKTablosu()
    test_ik(t)
    test_ik_tutma(t)
    ozet(t)
    ozet(t, 4)
