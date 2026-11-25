#!/usr/bin/env python3
"""Raspberry Pi 5 -> STM32 robot kol seri bağlantısı.

Kullanım:
    python3 kol_seri.py              # etkileşimli: komutları elle yaz
    python3 kol_seri.py --demo       # kısa bir hareket dizisi çalıştırır

Kod içinde kullanım:
    from kol_seri import Kol
    kol = Kol()
    kol.git(90, 110, 75, 30)          # hareket bitene kadar bekler
    print(kol.konum())

Pi 5 ayarı (bir kez): sudo raspi-config -> Interface Options -> Serial Port
  login shell: Hayır, serial hardware: Evet. Sonra yeniden başlat.
GPIO14/15 Pi 5'te /dev/ttyAMA0 olarak görünür.
"""
import argparse
import time

import serial

SERVO_MIN = (5, 50, 55, 0)
SERVO_MAX = (175, 145, 110, 135)


class KolHatasi(Exception):
    pass


class Kol:
    def __init__(self, port="/dev/ttyAMA0", baud=115200):
        self.s = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.1)
        self.s.reset_input_buffer()
        self._tampon = b""           # zaman aşımında gelen yarım satır burada bekler
        self._hareket_bitti = True   # son hareketin DONE'u geldi mi

    def _satir(self, zaman_asimi):
        bitis = time.time() + zaman_asimi
        while time.time() < bitis:
            self._tampon += self.s.readline()
            if self._tampon.endswith(b"\n"):
                satir = self._tampon.decode(errors="replace").strip()
                self._tampon = b""
                if satir:
                    return satir
        raise KolHatasi(f"{zaman_asimi:.1f} s içinde cevap gelmedi")

    def _gonder(self, komut, zaman_asimi=1.0):
        self.s.write((komut + "\n").encode())
        while True:
            cevap = self._satir(zaman_asimi)
            # bekle=False ile başlatılmış hareketin DONE'u ya da kart yeniden
            # başlayınca gelen READY bu komutun cevabı değildir
            if cevap == "DONE":
                self._hareket_bitti = True
            elif cevap != "READY":
                break
        if cevap.startswith("ERR"):
            raise KolHatasi(f"{komut!r} -> {cevap}")
        return cevap

    def ping(self):
        return self._gonder("P") == "PONG"

    def konum(self):
        parca = self._gonder("S").split(",")   # POS,a0,a1,a2,a3,DURUM
        return [int(x) for x in parca[1:5]], parca[5]

    def dur(self):
        cevap = self._gonder("X")
        self._hareket_bitti = True   # X'ten sonra STM32 DONE göndermez
        return cevap

    def git(self, taban, omuz, dirsek, kiskac, sure_ms=None, bekle=True):
        acilar = [round(taban), round(omuz), round(dirsek), round(kiskac)]
        for i, a in enumerate(acilar):
            if not SERVO_MIN[i] <= a <= SERVO_MAX[i]:
                raise KolHatasi(f"kanal {i}: {a} derece sınır dışı "
                                f"({SERVO_MIN[i]}-{SERVO_MAX[i]})")
        komut = "M," + ",".join(map(str, acilar))
        if sure_ms:
            komut += f",{int(sure_ms)}"
        self._gonder(komut)                      # OK beklenir
        self._hareket_bitti = False
        if bekle:
            self.bekle(sure_ms)

    def ev(self, bekle=True):
        self._gonder("H")
        self._hareket_bitti = False
        if bekle:
            self.bekle()

    def bekle(self, sure_ms=None):
        """Son hareketin DONE'unu bekler; zaten geldiyse hemen döner."""
        while not self._hareket_bitti:
            cevap = self._satir((sure_ms or 10000) / 1000 + 2.0)
            if cevap == "DONE":
                self._hareket_bitti = True
            elif cevap != "READY":
                raise KolHatasi(f"DONE beklenirken: {cevap}")


def demo(kol):
    print("ping:", kol.ping())
    print("konum:", kol.konum())
    adimlar = [
        (90, 90, 90, 0),
        (60, 110, 80, 0),
        (60, 110, 80, 100),
        (120, 100, 95, 100),
        (120, 100, 95, 0),
        (90, 90, 90, 0),
    ]
    for a in adimlar:
        t = time.time()
        kol.git(*a)
        print(f"{a} tamam ({time.time() - t:.2f} s)")


def etkilesimli(kol):
    print("Komut yaz (P, S, X, H, M,90,110,75,30[,ms]). Çıkmak için q.")
    while True:
        k = input("> ").strip()
        if k.lower() == "q":
            break
        if not k:
            continue
        kol.s.write((k + "\n").encode())
        try:
            cevap = kol._satir(1.0)
            print(cevap)
            if cevap == "OK":                  # hareket başladıysa DONE'u da bekle
                print(kol._satir(12.0))
        except KolHatasi as e:
            print("!", e)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--demo", action="store_true")
    arg = ap.parse_args()
    kol = Kol(arg.port)
    try:
        demo(kol) if arg.demo else etkilesimli(kol)
    except KeyboardInterrupt:
        kol.dur()
