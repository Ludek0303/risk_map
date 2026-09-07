# Mapowanie terenu

`mapping_capture.py` zapisuje podgląd `live_map.jpg` podczas lotu. Po
rozbrojeniu drona dopasowuje wspólne punkty zdjęć (SIFT + RANSAC), wyrównuje
całą sesję z odniesieniem do GPS i zapisuje `final_map.png`. Opcja
`--no-final-map` wyłącza obliczenia końcowe. Obliczenia mogą potrwać kilka
minut; nie blokują wcześniejszego podglądu podczas lotu.

Mapa wybiera fragmenty najbliższe środkom kadrów zamiast uśredniać
przesunięte obrazy. Odrzuca niespójne dopasowania i zbyt małe, odłączone
grupy zdjęć; brak pokrycia pozostaje czarny. Większe grupy bez wzajemnych
dopasowań są rozmieszczane na podstawie GPS. Widoczne szwy nadal są
możliwe. Model zakłada płaski teren i kamerę skierowaną w dół, nie zastępuje
rekonstrukcji 3D OpenDroneMap.

`images/` zawiera zdjęcia z symulowanymi wadami kamery do fotogrametrii;
`mosaic_images/` zachowuje obrazy użyte w podglądzie, z pozycją, wysokością
względną i kierunkiem w EXIF, do odtworzenia mapy końcowej. Zwiększa to
zapotrzebowanie na dysk. Wysokość względna zakłada teren na poziomie startu.

Odtworzenie istniejącej sesji bez uruchamiania symulatora:

```sh
python3 rebuild_mosaic.py captures/2026-09-06_214314 \
  --polygon missions/survey_45m_262x147_area.json \
  --estimate-heading --align \
  --out captures/2026-09-06_214314/map_aligned.png
```

`--estimate-heading` jest potrzebne tylko dla starych zdjęć bez kierunku
w EXIF. Przybliża kierunek kamery kierunkiem ruchu, co jest niedokładne przy
zakrętach i locie bokiem. Nowe sesje zapisują kierunek. Bez `--align` skrypt
wykonuje wyłącznie projekcję GPS. Domyślnie wybiera `mosaic_images/`, a dla
starych sesji korzysta z `images/`.

Odtwarzanie wymaga Python, NumPy, OpenCV z SIFT i piexif. Przechwytywanie
dodatkowo wymaga pymavlink, bibliotek Gazebo i działającej symulacji.

Testy: `python3 -m unittest test_live_mosaic.py`.
