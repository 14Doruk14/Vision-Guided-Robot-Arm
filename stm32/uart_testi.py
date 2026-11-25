#!/usr/bin/env python3
"""Pi <-> STM32 UART testi, adım adım. Pi'de çalıştır.

    python3 uart_testi.py --geri-donus   # 1. aşama: yalnız Pi, GPIO14-GPIO15 kablo ile kısa devre
    python3 uart_testi.py                # 2. aşama: STM32 bağlı, servo gücü KAPALI olabilir
    python3 uart_testi.py --hareket      # 3. aşama: servo gücü açık, taban 90 -> 100 -> 90

Her adım TAMAM ya da HATA yazar; ilk hatada durur ve olası nedeni söyler.
"""
import argparse
import sys
import time

import serial

from kol_seri import Kol, KolHatasi


def adim(ad, kosul, ipucu=""):
    if kosul:
        print(f"TAMAM  {ad}")
        return
    print(f"HATA   {ad}")
    if ipucu:
        print("       olası neden:", ipucu)
    sys.exit(1)


def ham(kol, komut, zaman_asimi=1.0):
    """Komutu gönderir, ilk cevap satırını döndürür (hata cevapları dahil)."""
    kol.s.write((komut + "\n").encode())
    try:
        return kol._satir(zaman_asimi)
    except KolHatasi:
        return None


def geri_donus(port):
    s = serial.Serial(port, 115200, timeout=0.5)
    s.reset_input_buffer()
    s.write(b"merhaba\n")
    gelen = s.readline()
    print("gelen:", gelen)
    adim("Pi UART geri dönüş (GPIO14 -> GPIO15)", gelen == b"merhaba\n",
         "seri port açık değil (raspi-config) ya da kablo GPIO14/15'e (pin 8/10) takılı değil")


def stm32(port, hareket):
    kol = Kol(port)

    print("kartın RESET düğmesine bas (READY bekleniyor, 5 s)...")
    try:
        ilk = kol._satir(5.0)
    except KolHatasi:
        ilk = None
    print("gelen:", ilk)
    if ilk != "READY":
        print("UYARI  READY gelmedi; reset atlanmış olabilir, devam ediliyor")

    c = ham(kol, "P")
    print("P ->", c)
    adim("P -> PONG", c == "PONG",
         "cevap yok: TX/RX çapraz mı (Pi pin 8 -> PA3, pin 10 -> PA2), GND ortak mı, "
         "CubeMX'te USART2 global interrupt açık mı, ReceiveToIdle_DMA çağrılıyor mu. "
         "Bozuk karakter: baud 115200 ve saat 168 MHz mi")

    c = ham(kol, "S")
    print("S ->", c)
    adim("S -> POS,90,90,90,0,IDLE", c == "POS,90,90,90,0,IDLE")

    for komut, beklenen in (("Z", "ERR,UNKNOWN"), ("M,90,90", "ERR,FORMAT"),
                            ("M,90,90,90,abc", "ERR,FORMAT"), ("M,200,90,90,0", "ERR,RANGE,0"),
                            ("M,90,90,30,0", "ERR,RANGE,2"), ("A" * 60, "ERR,OVERFLOW")):
        c = ham(kol, komut)
        print(f"{komut[:20]} -> {c}")
        adim(f"{komut[:20]} -> {beklenen}", c == beklenen)

    # 20 ping arka arkaya: tampon başa sarmasını (128 bayt) da geçer
    sayi = sum(ham(kol, "P") == "PONG" for _ in range(20))
    adim(f"20 ping arka arkaya ({sayi}/20)", sayi == 20,
         "bazıları kayboluyorsa RxEventCallback'teki başa sarma kısmına bak")

    if not hareket:
        print("\nHareket testi atlandı. Servo gücünü açıp --hareket ile tekrar çalıştır.")
        return

    print("\nTaban 90 -> 100 derece, 1,5 s (kol yalnız tabanda 10 derece dönmeli)")
    t = time.time()
    kol.git(100, 90, 90, 0, sure_ms=1500)
    sure = time.time() - t
    adim(f"M -> OK ... DONE ({sure:.2f} s)", 1.3 < sure < 2.5)
    c = kol.konum()
    adim(f"S -> {c}", c == ([100, 90, 90, 0], "IDLE"))

    kol.git(90, 90, 90, 0, sure_ms=1500, bekle=False)
    c = ham(kol, "M,80,90,90,0")
    adim(f"hareket sürerken M -> {c}", c == "ERR,BUSY")
    kol.bekle(1500)
    adim("geri dönüş tamam", kol.konum() == ([90, 90, 90, 0], "IDLE"))

    kol.git(100, 90, 90, 0, sure_ms=3000, bekle=False)
    time.sleep(1.0)
    c = kol.dur()
    durdugu = kol.konum()
    print("X ->", c, "konum:", durdugu)
    adim("X -> STOPPED, ortada kaldı", c == "STOPPED" and 90 < durdugu[0][0] < 100
         and durdugu[1] == "IDLE")
    kol.git(90, 90, 90, 0, sure_ms=1000)
    print("\nBütün UART testleri geçti.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--geri-donus", action="store_true", help="yalnız Pi, TX-RX kısa devre")
    ap.add_argument("--hareket", action="store_true", help="servo gücü açık, küçük hareket testi")
    arg = ap.parse_args()
    if arg.geri_donus:
        geri_donus(arg.port)
    else:
        stm32(arg.port, arg.hareket)
