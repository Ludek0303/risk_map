Gotowy plik do **testu importu w Mission Planner**:
[`missions/suas_2026_test.waypoints`](missions/suas_2026_test.waypoints).
Zawiera jeden start, 3 pełne pętle, riskmapping, powrót i jedno lądowanie.
Wczytaj go w trybie MISSION. Granicę z `suas_2026_test_fence.waypoints`
wczytuje się osobno w FENCES. Plik testowy używa przykładowych współrzędnych
oraz home `[47.397742, 8.545594, 500 m AMSL]`, a nie odczytu GPS drona.
Import obu plików sprawdzono czytnikiem lokalnej instalacji Mission Planner.
Do wygenerowania misji z aktualnym GPS użyj poniższej komendy.
Starsze sekcje README opisują również historyczne plany; ich wygenerowane
pliki usunięto podczas porządkowania katalogu.

## Jedna misja SUAS 2026: pętle → riskmapping

Edytuj **`missions/suas_2026_config.json`** i uruchom:

```bash
python3 generate_suas_mission.py
```

Generator odczyta home z GPS podłączonego drona i zapisze jedną ciągłą misję
**`missions/suas_2026.waypoints`** do Mission Planner:
start → liczba pełnych okrążeń z pola `laps` po `lap_points` → zmiana wysokości
w ostatnim punkcie pętli → dolot do obszaru `area_points` → riskmapping →
powrót do home → lądowanie. Między etapami nie ma lądowania ani wczytywania
kolejnej misji. Wysokości i trzy prędkości ustawiasz w tym samym JSON.
Przykładowe współrzędne należy zastąpić własnymi punktami terenu zawodów;
profil nie deklaruje zgodności z całym regulaminem zawodów.

W Mission Planner w trybie MISSION wczytaj **tylko `suas_2026.waypoints`**.
Pliki `suas_2026.json` (raport) i `suas_2026_area.json` (obszar zdjęć) są
pomocnicze. `suas_2026_fence.waypoints` zawiera granicę do osobnego trybu
FENCES i nie jest drugą misją lotniczą. Zdjęcia nadal wykonuje skrypt capture.

Opcjonalne argumenty, np. `--connection`, `--mission-config` i `--out`,
działają tak jak w `generate_survey_mission.py`. Stare wygenerowane przykłady zostały usunięte z `missions/`.

# Mapowanie terenu

`mapping_capture.py` zapisuje szybki podgląd `live_map.jpg` podczas lotu.
Po rozbrojeniu, przy co najmniej sześciu zdjęciach, uruchamia rekonstrukcję
końcową i zapisuje `final_map.png` oraz raport `final_map.json`.
Opcja `--no-final-map` wyłącza obliczenia po lądowaniu.

## Dopasowanie zdjęć podczas lotu

`mapping_capture.py` domyślnie rozpoczyna obliczenia w tle po zapisaniu
**pierwszego czystego zdjęcia**. Dla pierwszego liczy cechy SIFT;
od drugiego sprawdza pary zdjęć według GPS i kolejności wykonania.
Nie wymaga to nowej komendy ani zmiany misji QGC.

Praca odbywa się w osobnym procesie o obniżonym priorytecie i z dwoma
wątkami OpenCV. Przechwytywanie przekazuje tylko nazwy zapisanych plików
i współrzędne, nie czeka na obliczenia. Nowo napływające zdjęcia mają
pierwszeństwo przed kolejką porównań. Worker przechowuje w RAM cechy
maksymalnie 12 zdjęć, a pozostałe odczytuje z dysku.

Postęp znajdziesz w katalogu sesji:

- `inflight_matching.log` — policzone cechy i pary, wielkość kolejki;
- `.inflight_matches.sqlite3` — trwały cache cech oraz udanych i odrzuconych par;
- `inflight_report.json` — podsumowanie przy normalnym zakończeniu workera;
- `inflight_reuse.json` — liczba wyników cache użytych przy końcowym dopasowaniu.

Po rozbrojeniu worker kończy bieżącą operację, a końcowa rekonstrukcja
wykorzystuje zapisane wyniki. Uzupełnia brakujące zdjęcia, pary i wyszukiwanie
podobieństwa wizualnego, po czym wyrównuje kamery i składa mapę. Nie odrzuca
zdjęć tylko dlatego, że worker nie zdążył ich przetworzyć podczas lotu.
Cache uwzględnia ścieżkę, rozmiar i czas modyfikacji zdjęcia, profil kamery
i rodzaj dopasowania. Przerwanie procesu zachowuje zatwierdzone wyniki;
awaria workera nie zatrzymuje zapisu zdjęć.

`--no-inflight-matching` wyłącza obliczenia podczas lotu.
`--no-final-map` również je wyłącza. `--final-map-fast` dobiera wariant
dopasowania zgodny z szybką rekonstrukcją. Cache zwiększa zajętość dysku;
obliczenia w tle skracają pracę po lądowaniu, ale nie gwarantują czasu
12 minut ani pełnego pokrycia obszaru.

Weryfikacja na pierwszych 12 czystych zdjęciach sesji `2026-09-08_095302`:
worker przygotował 12 zestawów cech i sprawdził 66 par (50 zaakceptowanych).
Końcowe dopasowanie wykorzystało wszystkie 12/66 wyników z cache i trwało
0,096 s. To odtworzenie napływu zdjęć, nie pełny lot ani pomiar czasu całej mapy.
Raport: `captures/2026-09-08_095302/diagnostics/inflight_replay_12/replay_result.json`.

## Telemetria LR900 przez Mission Planner

`mapping_capture.py` może pobierać pozycję, wysokość, uzbrojenie i orientację
z tego samego strumienia MAVLink, który Mission Planner otrzymuje przez
MicoAir LR900. LR900 pozostaje otwarty wyłącznie przez Mission Planner;
program mapujący nie otwiera portu COM radia.

W Mission Planner po połączeniu z dronem otwórz **Mavlink Forwarding**
w Advanced Tools, wybierz UDP i jako cel wpisz `127.0.0.1`, port `14551`.
Następnie, na tym samym komputerze, uruchom:

```sh
python3 mapping_capture.py \
  --mission-planner-udp-port 14551 \
  --polygon missions/survey_45m_262x147_8min_area.json \
  --capture-interval-m 5 --max-tilt-deg 8 --min-altitude-m 42.75
```

Schemat przepływu to `dron → LR900 → Mission Planner → UDP localhost →
mapping_capture.py`. Misja nadal jest wykonywana i monitorowana w Mission
Planner. Program nasłuchuje tylko na `127.0.0.1` albo `localhost`, więc
telemetria nie jest wystawiana do sieci lokalnej.

W tym trybie automatyczne kierowanie gimbalem jest wyłączone. Przed startem
ustaw kamerę pionowo w dół w Mission Planner lub w misji. Jeśli świadomie
włączyłeś w Mission Planner przekazywanie poleceń z UDP do drona, możesz
dodać `--relay-gimbal-control`; domyślnie nie jest to potrzebne do odbioru
pozycji i robienia zdjęć.

LR900 działa jako przezroczysty łącznik danych; producent podaje domyślne
57600 Bd dla USB/UART i zaleca szybki tryb łącza do kontrolera lotu.
Ustawienia serialu po stronie drona muszą nadal odpowiadać radio-modułowi.
Zobacz [instrukcję LR900](https://micoair.com/radio_telemetry_lr900f/) oraz
[opis lokalnego MAVLink Forwarding w Mission Planner](https://ardupilot.ardupilot.org/planner/docs/common-mp-tools.html).

## Kamera RTSP i LR900 po USB

Adres `rtsp://192.168.144.25:8554/main.264` jest prawidłowym strumieniem
RTSP używanym przez kamery SIYI, między innymi A8 i ZR10. Podłącz laptop
do tej samej podsieci `192.168.144.x`; dokumentacja SIYI/ArduPilot zaleca,
aby pierwsze trzy oktety adresu laptopa i kamery były takie same.
[Konfiguracja SIYI](https://en.ardupilot.org/sub/docs/common-siyi-zr10-gimbal.html)

Do bezpośredniego odbioru obrazu z RTSP oraz telemetrii z LR900 po USB użyj:

```sh
python3 mapping_capture.py \
  --rtsp-url rtsp://192.168.144.25:8554/main.264 \
  --lr900-usb-port /dev/ttyUSB0 --lr900-baud 57600 \
  --camera-hfov-deg TWOJ_RZECZYWISTY_HFOV \
  --polygon missions/survey_45m_262x147_8min_area.json \
  --capture-interval-m 5 --max-tilt-deg 8 --min-altitude-m 42.75
```

Zastąp `/dev/ttyUSB0` właściwym portem LR900; na Windows będzie to np.
`COM3`. Ustal go komendą `ls /dev/ttyUSB* /dev/ttyACM*` albo w menedżerze
urządzeń. Przed lotem sprawdź obraz np. w VLC, otwierając ten sam adres
RTSP. Program czeka do 15 s na pierwszą klatkę i kończy się z jasnym błędem,
jeśli strumień nie działa; potem automatycznie próbuje ponownego połączenia.
Zachowuje wyłącznie najnowszą klatkę, więc nie buduje opóźnienia obrazu.

`TWOJ_RZECZYWISTY_HFOV` musi być poziomym polem widzenia szerokiego obiektywu
bez zoomu. Jest potrzebny do geometrii mapy; nie wpisuj wartości dla trybu
zoom. Domyślne 114,6° pochodzi z kamery symulatora i program wypisze
ostrzeżenie, jeśli pozostanie przy RTSP. Zdjęcia z RTSP nie są przycinane
i nie przechodzą przez sztuczne pogorszenie obrazu symulatora. EXIF sesji
zapisuje źródło jako SIYI/RTSP-main.264, a `capture_config.json` zachowuje
użyte parametry kamery.

Bezpośredni wariant USB oznacza, że `mapping_capture.py` jest jedynym
programem otwierającym port LR900. Nie podłączaj równocześnie Mission Planner
do tego samego portu COM. Jeżeli Mission Planner ma działać równolegle,
użyj wcześniejszego wariantu `--mission-planner-udp-port`, a on jeden otworzy
port LR900 i przekaże MAVLink lokalnie.

## Pamięć i tryb szybszych obliczeń

Naprawiono przerwanie procesu przez system z powodu braku pamięci (OOM)
na sesji 468 zdjęć. Składanie wczytuje pełne obrazy pojedynczo, przechowuje
tylko małe podglądy do wyboru szwów i ogranicza liczbę bloków wyrównywania
jasności do 2048 dla sesji do 2048 ujęć. W poprzedniej wersji stałe bloki
32 px powodowały bardzo duży, gęsty układ równań. Sposób alokacji potwierdza
[kod OpenCV](https://github.com/opencv/opencv/blob/4.x/modules/stitching/src/exposure_compensate.cpp).
W logu pojawia się teraz postęp składania poszczególnych zdjęć.

Opcja `rebuild_mosaic.py --align --fast` wybiera do 220 zdjęć rozłożonych
po całej trasie, używa indeksu FLANN do wyszukiwania cech i mniejszej liczby par. Zachowuje oryginalne
zdjęcia oraz oddzielny cache `.orthomosaic_fast_matches.npz`.
Przy przechwytywaniu włącz ten wariant przez `mapping_capture.py --final-map-fast`.
Nie oznacza to gwarancji pełnego pokrycia ani zakończenia w 12 minut.
Pierwszy test starszego wariantu (180 zdjęć, 2000 cech) na sesji `2026-09-08_095302` trwał 155,8 s bez cache,
ale pokrył tylko 82% płótna; wynik jest w `fast_test/final_map.png` tej sesji.
Ten pomiar wykonano jeszcze przed poprawką pamięci etapu składania.
Aktualny wariant (220 zdjęć, 3500 cech, FLANN) na tej sesji: 265,3 s
od początku bez cache, 93,8% pokrycia płótna, błąd 0,46 px i 0,60 GiB
maksymalnego RSS. Raport i obraz: `fast_verified/final_map.json` oraz
`fast_verified/final_map.png`; log: `fast_verified.log`.
Nie spełnia to jeszcze celu pełnej mapy w 12 minut od startu przy locie
8-minutowym. Do pełnego pokrycia tej sesji wybierz obecnie zwykłe `--align`.

Pełny przebieg po naprawie pamięci, dla 468 zdjęć tej sesji, zakończył się
poprawnie: 99,9% pokrycia płótna, błąd reprojekcji 0,50 px, maksymalny RSS
3 778 884 KiB (3,60 GiB). Czas 293,5 s obejmuje odczyt gotowego cache
dopasowań, wyrównanie i składanie. Nie obejmuje ponownego wyszukiwania par.
Log z pomiarem `/usr/bin/time -v`: `captures/2026-09-08_095302/rebuild_verified.log`.
Optymalizator osiągnął limit iteracji; raport uczciwie zapisuje
`metric_solver_converged: false`. Poszczególne kamery nadal przechodzą
kontrole błędu i geometrii. Przy drzewach pozostają artefakty modelu płaskiego gruntu.

Po próbie rekonstrukcji uruchomionej automatycznie powstaje również
`mission_timing.json`: czas od pierwszego zaobserwowanego uzbrojenia
do rozbrojenia i do zakończenia tworzenia mapy, status powodzenia oraz
informacja o zmieszczeniu się w 720 sekundach. Uruchom przechwytywanie
przed uzbrojeniem, żeby pomiar obejmował cały lot. Jest to pomiar czasu,
nie mechanizm przerywania obliczeń po upływie limitu.

## Misja QGroundControl do zdjęć

Wariant na **około 8 minut**: `missions/survey_45m_262x147_8min.plan`.
Zachowuje obszar 147 × 262 m i wysokość 45 m. Ma 8 pasów co 20,4 m,
prędkość 7,4 m/s, postoje 1 s i zdjęcia co 5 m. Nominalne pokrycie
wynosi 74% między pasami i 94% wzdłuż lotu. Krótszy czas oznacza
mniejszy zapas pokrycia i większą podatność na rozmycie ruchowe niż
w wolniejszym wariancie `_photo`.
Szacunek uwzględnia start, lądowanie, postoje i rozpędzanie/hamowanie
z przyspieszeniem 2 m/s² oraz prędkość pionową 1,5 m/s. Te wartości
są założeniami obliczeń, nie zmianą parametrów PX4; rzeczywisty czas
wymaga sprawdzenia w locie i nie jest gwarantowany co do sekundy.

```sh
python3 mapping_capture.py \
  --polygon missions/survey_45m_262x147_8min_area.json \
  --capture-interval-m 5 --max-tilt-deg 8 --min-altitude-m 42.75
```

Odtworzenie wariantu:

```sh
python3 generate_survey_mission.py --width-m 147 --height-m 262 \
  --altitude-m 45 --sidelap 0.70 --cruise-speed 7.4 --settle-seconds 1 \
  --horizontal-accel-m-s2 2 --center-lat 37.412173071650805 \
  --center-lon -121.998878727967 --home-alt-amsl 38 \
  --out missions/survey_45m_262x147_8min.plan
```

Gotowy poprawiony plan: `missions/survey_45m_262x147_photo.plan`.
Otwórz go w QGC w widoku Plan. Poprzedni plik bez `_photo` zachowano.
Nowa misja dla tego samego prostokąta 147 × 262 m ma wysokość 45 m,
11 pasów co 14,3 m i prędkość 3 m/s. Prędkość jest ustawiona zarówno
w `hoverSpeed`, jak i w poleceniu `MAV_CMD_DO_CHANGE_SPEED`.
Na końcach pasów są postoje 3 s; start i końcowe zniżanie odbywają się
nad środkiem obszaru, z powrotem poziomym na wysokości przelotowej.
Planowane miejsce startu musi odpowiadać pozycji drona w symulacji.

Rozstaw uwzględnia przycięcie kamery 1280 × 720 do 1024 × 720
i używa krótszego boku śladu zdjęcia, żeby nie zakładać konkretnego yaw.
Nominalne pokrycie na płaskim gruncie wynosi około 82% między pasami
i 94% wzdłuż lotu przy zdjęciach co 5 m. Punkty są odsunięte 2 m od
granicy do wnętrza, aby warunek wejścia w obszar nie blokował zdjęć.
Przechylenie kamery, opóźnienia telemetrii i pominięte klatki mogą
zmniejszyć faktyczne pokrycie.

Przed uzbrojeniem uruchom zapis zdjęć i ustawienie gimbala pionowo w dół:

```sh
python3 mapping_capture.py \
  --polygon missions/survey_45m_262x147_photo_area.json \
  --capture-interval-m 5 --max-tilt-deg 8 --min-altitude-m 42.75
```

Zdjęcia Gazebo zapisuje ten skrypt; samo wczytanie planu do QGC nie
uruchamia zapisu. Próg wysokości pomija większość wznoszenia i zniżania,
a próg przechylenia odrzuca mocno pochylone ujęcia. Mapa podglądowa powstaje
w trakcie lotu, końcowa po rozbrojeniu.

Szacowany lot trwa 20,3 min, obejmując 3,28 km trasy, postoje i założone
1,5 m/s wznoszenia oraz opadania. Rozpędzanie, hamowanie i ustawienia PX4
wydłużą ten czas. Około 572 zdjęcia przypadają na same pasy; dojazdy oraz
pomijane ujęcia zmienią liczbę zapisanych plików. Nie wykonano jeszcze
lotu kontrolnego tej nowej misji.

Generator działa też bez uruchomionej symulacji:

```sh
python3 generate_survey_mission.py --width-m 147 --height-m 262 \
  --altitude-m 45 --center-lat 37.412173071650805 \
  --center-lon -121.998878727967 --home-alt-amsl 38 \
  --out missions/survey_45m_262x147_photo.plan
```

Zapisuje plan, wielokąt `_area.json` i raport `.json` z czasem lotu,
pokryciem i odstępem zdjęć. `--frontlap` może zmniejszyć odstęp zdjęć;
generator wypisuje odpowiadające mu polecenie uruchomienia zapisu.
Domyślne parametry dotyczą kamery x500_gimbal, nie dowolnej kamery.
Format i polecenie prędkości sprawdzono w dokumentacji
[QGC](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html)
i [MAVLink](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED).

## Mapa końcowa

Korony drzew i inne obiekty z paralaksą otrzymują dodatkową ochronę przed
rozmyciem: po wyrównaniu jasności program wykrywa duże różnice między
nakładającymi się zdjęciami, łączy je w obszary i wybiera dla każdego
jedno możliwie centralne ujęcie obejmujące przynajmniej 98% obszaru.
Wnętrze pochodzi z tego zdjęcia, z przejściem około 2 pikseli przy granicy.
Obszary bez odpowiedniego zdjęcia zachowują standardowe łączenie.
To wykrywanie różnic obrazów, a nie semantyczne rozpoznawanie drzew.
Nie dorabia tekstur ani kolorów i nie odtwarza wysokości koron — ogranicza
przezroczyste, podwójne kontury wynikające z mieszania przesuniętych ujęć.
Wynik dla sesji `2026-09-08_095302` zapisano jako `final_map.png`
i `final_map_trees.png`, a poprzedni zachowano w `final_map_before_trees.png`.
Ochrona objęła 102 obszary; maska pokrycia pozostała identyczna (99,9%).
Najbardziej widoczna poprawa dotyczy mniejszych koron; duże grupy drzew
bez jednego ujęcia obejmującego całość mogą nadal mieć artefakty.

`orthomosaic.py` wykonuje następujące kroki:

1. Opcjonalnie odwraca znane zniekształcenie i winietowanie dodane przez
   `camera_realism.py`. Profil jest jawny, nie stosuje się go do innych kamer.
2. Wykrywa SIFT, szuka par według GPS i podobieństwa wizualnego oraz
   sprawdza homografie przez RANSAC. Obsługuje perspektywę przechylonych kadrów.
3. Wstępnie optymalizuje homografie w największej połączonej rekonstrukcji.
   Następnie optymalizuje pozycje i obroty skalibrowanych kamer oraz wspólne
   punkty gruntu (rzadkie bundle adjustment ze stratą odporną na błędy).
   Nie dokleja niezależnych fragmentów na podstawie samego GPS.
4. Wyznacza normalną płaszczyzny gruntu z kalibracji i ruchu kamer, usuwa
   perspektywę oraz dopasowuje pozycje kamer do GPS. Środek zdjęcia przechylonej
   kamery nie jest traktowany jako punkt bezpośrednio pod dronem.
5. Odrzuca źródła z niedostateczną liczbą obserwacji, błędem reprojekcji
   powyżej 2,5 px, błędem pozycji powyżej 10 m lub przechyleniem powyżej 20°.
   Spójne dopasowania muszą obejmować co najmniej 30% powierzchni kadru;
   zapobiega to rozciąganiu małego poprawnego fragmentu na cały obraz.
6. Wyrównuje ekspozycję, wybiera granice łączenia metodą graph cut i stosuje
   mieszanie wielopasmowe. PNG ma przezroczystość w miejscach bez danych.

Jest to rekonstrukcja jednej płaszczyzny gruntu, inspirowana etapami
fotogrametrii ODM, a nie pełna rekonstrukcja 3D. Drogi i płaskie powierzchnie
można wyrównać; wysokie drzewa i budynki mogą nadal wykazywać paralaksę.
Dokładność dopasowania zdjęć nie oznacza takiej samej dokładności GPS.
Raport JSON podaje obie miary osobno i wymienia odrzucone fotografie.

Obliczenia dla kilkuset zdjęć mogą potrwać kilka minut i korzystają z RAM
proporcjonalnego do rozmiaru rekonstrukcji. Cache `.orthomosaic_matches.npz`
pozwala ponawiać obliczenia bez ponownego wyszukiwania punktów; jest sprawdzany
względem źródeł i profilu kamery. Nie potrzeba Dockera. Optymalizacja metryczna wymaga SciPy i działa
w osobnym procesie. Jeśli pakiety użytkownika kolidują z systemowym SciPy,
worker próbuje stosu systemowego przez `python -s`. Można też użyć spójnego
środowiska wirtualnego z zależnościami z `requirements-mapping.txt`.
Każda nowa rekonstrukcja wypisuje zmierzony czas i zapisuje go w raporcie
`final_map.json`: `processing_seconds` oraz `stage_seconds` (dopasowanie,
wyrównanie i składanie). Czas ten dotyczy obliczeń po locie, nie samego lotu.

Pomiar 2026-09-08 na Intel Core Ultra 5 225H, dla 324 zdjęć starej sesji
1024 × 720 i mapy 1256 × 2177, **bez cache dopasowań**: **472,2 s
(7 min 52 s)**. Dopasowanie zajęło 413,0 s, wyrównanie 18,8 s,
a składanie i zapis obrazu 40,5 s. Raport pomiaru:
`captures/2026-09-06_214314/diagnostics/benchmark/timing.json`.
To czas naszego generatora, nie pełnego ODM. Nowa misja dostarczy więcej
zdjęć; jej czasu rekonstrukcji jeszcze nie zmierzono i nie należy
przeliczać go liniowo z liczby zdjęć.

## Odtwarzanie zapisanej sesji

Dla tej starej sesji zdjęcia zawierają sztuczne wady z `camera_realism.py`:

```sh
python3 rebuild_mosaic.py captures/2026-09-06_214314 \
  --polygon missions/survey_45m_262x147_area.json \
  --align --camera-profile synthetic-cgo3 \
  --out captures/2026-09-06_214314/final_map.png
```

Dla nowych sesji stosuj `--align` bez `--camera-profile`: skrypt wybiera
`mosaic_images/`, gdzie są obrazy bez sztucznych wad. Dla starszych sesji
korzysta z `images/`. Kierunek kamery jest odzyskiwany z obrazów; rekonstrukcja
nie wymaga kierunku w EXIF ani `--estimate-heading`.

Bez `--align` dostępna jest prosta projekcja GPS. Tylko w tym trybie stare
zdjęcia bez kierunku wymagają jawnego `--estimate-heading` (kierunek ruchu).

`images/` przechowuje zdjęcia z symulowanymi wadami do testów fotogrametrii;
`mosaic_images/` zachowuje czyste obrazy z geotagami do końcowej mapy.
EXIF GPSAltitude w tych sesjach oznacza wysokość względem startu, zakładając
teren na tym samym poziomie. Dodatkowy zapis zwiększa zużycie dysku.

## Zależności i weryfikacja

Odtwarzanie wymaga Python, NumPy, OpenCV z SIFT i modułem stitching oraz
piexif oraz SciPy. Przechwytywanie dodatkowo wymaga pymavlink, bibliotek Gazebo i
uruchomionej symulacji. `align_mosaic.py` to poprzedni model podobieństwa;
nie jest już używany do końcowej mapy.

```sh
python3 -m unittest test_inflight_matching.py test_generate_survey_mission.py test_orthomosaic.py test_live_mosaic.py
```

Testy sprawdzają odzyskiwanie geometrii przechylonych kamer, wspólne
wyrównanie, korekcję winietowania, wygładzanie różnic ekspozycji, odrzucanie zbyt małych obszarów
dopasowania oraz podgląd.

ODM: `--odm-orthophoto-resolution` podaje **cm/piksel**, a
`--mosaic-resolution` podaje **piksele/metr**. Przykładowo 5 cm/piksel to
20 pikseli/metr. Istniejący tryb ODM korzysta z `--fast-orthophoto`.

Źródła użyte przy implementacji:

- [OpenSfM: rekonstrukcja i bundle adjustment](https://opensfm.org/docs/reconstruction_module.html)
- [ODM: MVS texturing i seam leveling](https://github.com/OpenDroneMap/mvs-texturing)
- [OpenCV: rektyfikacja i dekompozycja homografii](https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html)
- [OpenCV: kompensacja ekspozycji, szwy i multiband blending](https://docs.opencv.org/4.x/d9/dd8/samples_2cpp_2stitching_detailed_8cpp-example.html)
- [ODM: jednostka rozdzielczości ortofotomapy](https://docs.opendronemap.org/arguments/orthophoto-resolution/)


## Misja z czterech narożników i okrążeń na początku

Generator `generate_survey_mission.py` przyjmuje `--mission-config` i domyślnie
tworzy plik Mission Planner dla ArduCoptera (`missions/survey.waypoints`).
Skopiuj `missions/suas_2026_config.json` i wpisz własne współrzędne:

Miejsce startu i powrotu (`home`) jest domyślnie odczytywane z aktualnej
pozycji GPS połączonego drona podczas generowania planu. Aby wygenerować
misję i geofence bez łączenia się z MAVLink (np. do samego podglądu punktów,
zanim dron będzie dostępny), podaj `--center-lat`/`--center-lon` (opcjonalnie
`--home-alt-amsl`) — te wartości posłużą jedynie jako placeholder w wierszu 0
pliku i jako punkt odniesienia lokalnej projekcji; nie przesuwają samych
punktów misji. Autopilot i tak ustala rzeczywiste home przy uzbrojeniu, więc
przed lotem wygeneruj misję ponownie z podłączonym dronem, aby wiersz 0
odpowiadał realnemu miejscu startu.

- `area_points`: dokładnie cztery narożniki obszaru zdjęć. Kolejność jest dowolna;
  generator uporządkuje je po obwodzie. Muszą tworzyć wypukły czworokąt
  (np. obrócony prostokąt lub trapez), bez powtórzeń i narożników współliniowych.
  Lokalna projekcja obsługuje obszary o przekątnej do 20 km, poza biegunami
  i bez przekraczania południka 180°.
- `lap_points`: dowolna liczba punktów początkowej pętli **w kolejności lotu**
  (nie ma ograniczenia do czterech).
  Powrót do pierwszego punktu jest automatyczny. Wymagane są przynajmniej
  dwa różne punkty przy dodatniej liczbie okrążeń.
- `laps`: liczba pełnych okrążeń, np. `3`; `0` pomija pętlę.

Przykład uruchomienia z dołączoną konfiguracją demonstracyjną:

```bash
python3 generate_survey_mission.py \
  --mission-config missions/suas_2026_config.json \
  --altitude-m 45 --lap-altitude-m 60 --cruise-speed 3 --lap-speed 8 \
  --out missions/four_points_survey.waypoints
```

`--altitude-m 45` ustawia wysokość przelotów nad obszarem zdjęć i powrotu,
a `--lap-altitude-m 60` wysokość startu, dolotu do pętli i wszystkich okrążeń.
Obie wysokości są w metrach względem home autopilota. Bez `--lap-altitude-m`
pętla korzysta z `--altitude-m`. Po ostatnim okrążeniu dron zmienia wysokość
w ostatnim punkcie pętli, a dopiero potem leci nad obszar zdjęć.
Przy `laps: 0` od razu startuje na wysokość zdjęć.
Generator wymaga połączenia MAVLink (domyślnie `udpin:127.0.0.1:14540`,
zmieniane przez `--connection`). Uruchom go przed startem drona.
Parametry pokrycia zdjęć `--sidelap`, `--frontlap` i postoje
`--settle-seconds` działają również w tym trybie.

Przebieg: pionowy start → dolot do pierwszego punktu pętli → zadana liczba
pełnych okrążeń → przeloty północ–południe przycięte do obszaru zdjęć →
powrót na wysokości przelotowej do `home` → lądowanie. Przykładowo dla
punktów A, B, C i dwóch okrążeń trasa pętli to A → B → C → A → B → C → A.
Okrążenia są zapisane jako zwykłe waypointy, więc widać wszystkie w Mission Planner.

Wyniki: `.waypoints` do otwarcia w Mission Planner, `_area.json` dla skryptu zdjęć
oraz `.json` z raportem uwzględniającym również długość i czas okrążeń.
Generator wypisuje gotową komendę `mapping_capture.py` dla tej misji.
Nie wysyła misji do drona ani nie uruchamia lotu.

Zdjęcia wykonuje osobno `mapping_capture.py`, gdy dron znajduje się wewnątrz
obszaru i spełnia warunki wysokości oraz nachylenia. Jeżeli pętla lub dolot
przecinają obszar zdjęć, zdjęcia mogą powstawać także na tych odcinkach.
Przykładowa pętla leży poza obszarem zdjęć.

Testy generatora:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest test_generate_survey_mission.py test_mission_geometry.py test_mission_planner_export.py
```

### Wczytanie do Mission Planner (ArduCopter)

W zakładce **PLAN / Flight Plan** kliknij prawym przyciskiem mapę, wybierz
**Load WP File** i otwórz wygenerowany plik `.waypoints`. Po sprawdzeniu
trasy użyj **Write WPs**, aby przesłać ją do autopilota.
Zapis jest tekstowy, z nagłówkiem `QGC WPL 110` — nazwa nagłówka jest częścią
formatu waypointów i nie oznacza, że plik jest przeznaczony wyłącznie do QGC.

Wiersz 0 zawiera planowane home: pozycję GPS odczytaną podczas generowania
oraz wysokość AMSL. Wiersze lotu mają wysokości względne względem home
ArduCoptera. Plan kończy się waypointem powrotu do odczytanych współrzędnych
oraz komendą LAND. Home autopilota ustalane przy uzbrojeniu jest odrębne od
wiersza home w pliku; generuj plan w miejscu, z którego dron będzie startował.

Dla ArduCoptera postój `--settle-seconds` jest zaokrąglany w górę do pełnych
sekund (maksymalnie 65535); raport uwzględnia wartość po zaokrągleniu.
Promień osiągnięcia waypointu wynika z ustawień ArduCoptera.
Eksport jest przeznaczony dla wielowirnikowca, nie ArduPlane ani Rovera.

Można jawnie wybrać `--format mission-planner`. Dotychczasowy eksport PX4/QGC
pozostaje dostępny przez `--format qgc` lub nazwę wyjściową kończącą się `.plan`.
Rozszerzenie musi pasować do wybranego formatu. Bez `--out` i `--format`
powstaje `missions/survey.waypoints`.

Eksport oraz kolejność i wysokości punktów sprawdzono testami, w tym odczytem
pliku przez `pymavlink.MAVWPLoader`. Nie sprawdzono wykonania w ArduCopter SITL
ani importu w uruchomionej aplikacji Mission Planner.

Źródła formatu i importu:
[format WPL](https://mavlink.io/en/file_formats/),
[Mission Planner — planowanie misji](https://ardupilot.org/planner/docs/common-planning-a-mission-with-waypoints-and-events.html).

### Prędkość przelotu

- `--cruise-speed 3`: prędkość nad obszarem zdjęć i powrotu do home; domyślnie 3 m/s.
- `--lap-speed 8`: prędkość początkowych okrążeń i dolotu do pierwszego punktu
  pętli, w m/s. Bez tej opcji stosowane jest `lap_speed_m_s` z JSON, a przy braku tego pola
  wartość `--cruise-speed`.
  Przy `laps: 0` ustawienie pętli nie jest używane.

Obie wartości muszą być dodatnie. Generator zapisuje komendy
`DO_CHANGE_SPEED`: pierwszą po starcie, a przy różnych prędkościach drugą
po pętli i zmianie wysokości, przed dolotem do obszaru zdjęć.
Przykładowo 8 m/s to 28,8 km/h, a 3 m/s to 10,8 km/h.
Są to zadane prędkości poziome względem ziemi; faktyczny lot uwzględnia
ograniczenia autopilota, przyspieszanie i postoje w waypointach.
Raport czasu uwzględnia oddzielne prędkości obu etapów.


### Granica lotu — flight boundary

Do konfiguracji JSON można dodać opcjonalne `flight_boundary`, np.:

```json
"flight_boundary": [
  [47.3974, 8.5448],
  [47.3974, 8.5475],
  [47.4001, 8.5475],
  [47.4001, 8.5448]
]
```

Wpisz od 3 do 70 punktów `[szerokość GPS, długość GPS]` **w kolejności po
obwodzie**, zgodnie lub przeciwnie do ruchu wskazówek zegara. Pierwszego
punktu nie trzeba powtarzać na końcu. Wielokąt może być wklęsły, ale nie
może przecinać sam siebie; powtórzone i kolejne współliniowe narożniki są
odrzucane. Jest to pozioma granica dozwolonego obszaru, niezależna od
czterech narożników obszaru zdjęć. Nie ustawia limitu wysokości.

Generator sprawdza, czy obszar zdjęć, home z GPS i cała trasa (także odcinki
między waypointami, doloty i powrót) mieszczą się w granicy. Przy błędzie
nie zapisuje nowych plików. Nie przelicza objazdu granicy. Kontrola dotyczy
geometrii planowanej trasy, bez zapasu na rzeczywisty tor lotu i FENCE_MARGIN.

Przykładowa konfiguracja `missions/suas_2026_config.json` zawiera już
`flight_boundary`. Komenda generowania misji pozostaje taka sama.
Dla `my_survey.waypoints` dodatkowo powstaje **`my_survey_fence.waypoints`**.
To osobny plik typu inclusion fence, a nie kolejna część trasy.

W Mission Planner:

1. Wczytaj i zapisz misję w trybie **MISSION**.
2. W **PLAN** przełącz listę typu planu na **FENCES**, użyj **Load WP File**
   dla pliku `_fence.waypoints`, następnie **Write**.
3. W **CONFIG → GeoFence** włącz ogrodzenie wielokątne i ustaw reakcję na
   przekroczenie granicy. Sam import pliku nie włącza geofence.

Dla eksportu `.plan` granica trafia do sekcji `geoFence` w pliku QGC.
Bez `flight_boundary` generator tworzy misję bez nowej granicy; nie usuwa
wcześniej zapisanych plików fence ani granic już wgranych do autopilota.

[Obsługa FENCES w Mission Planner](https://ardupilot.org/planner/docs/mission-planner-flight-plan.html).
Testy granic: `python3 -m unittest test_flight_boundary.py`.


### Wysokości i prędkości w JSON

W pliku konfiguracji można ustawić wszystkie pięć wartości:

```json
"lap_altitude_m": 65,
"riskmapping_altitude_m": 40,
"lap_speed_m_s": 8,
"riskmapping_speed_m_s": 3,
"transfer_speed_m_s": 6
```

Wysokości są w metrach względem home autopilota, prędkości poziome w m/s.
Powyższy przykład oznacza:

1. Start na 65 m, dolot do pętli i okrążenia z prędkością 8 m/s.
2. Po pętli zmiana wysokości do 40 m w jej ostatnim punkcie.
3. Przelot do pierwszego punktu riskmappingu na 40 m z prędkością 6 m/s.
4. Po dotarciu do tego punktu przełączenie na 3 m/s dla riskmappingu.
5. Powrót do home na wysokości 40 m z prędkością 3 m/s, następnie lądowanie.

Uruchomienie wymaga tylko wskazania konfiguracji i pliku wynikowego:

```bash
python3 generate_survey_mission.py \
  --mission-config missions/suas_2026_config.json \
  --out missions/my_survey.waypoints
```

Dołączona konfiguracja zawiera pięć pól; domyślnie ma 45 m i 3 m/s.
Parametry CLI mają pierwszeństwo przed JSON: `--lap-altitude-m`,
`--altitude-m`, `--lap-speed`, `--cruise-speed`, `--transfer-speed`.
Jeśli odpowiednich pól ani opcji nie podasz, riskmapping przyjmuje 45 m
oraz 3 m/s, a pętla i transfer korzystają z wartości riskmappingu.
Przy `laps: 0` pętla i transfer między etapami są pomijane.
Wszystkie pięć wartości musi być dodatnimi, skończonymi liczbami.

Raport zawiera ustawienia i długość transferu; szacunek czasu uwzględnia
każdą z trzech prędkości. Zdjęcia nadal uruchamia osobny skrypt według
obszaru GPS, więc może je wykonywać także w trakcie transferu przez obszar.


### Naprawa importu prędkości ułamkowej w Mission Planner

Jeżeli import `.plan` zgłasza `Input string '7.4' is not a valid integer`
dla `mission.cruiseSpeed`, przyczyną są typy pól w czytniku JSON Mission Planner.
Generator zapisuje teraz `cruiseSpeed` i `hoverSpeed` jako liczby całkowite
w metadanych. Dokładne prędkości, np. 7.4 m/s, pozostają w komendach
`DO_CHANGE_SPEED` oraz w raporcie. Metadane mogą więc pokazywać zaokrągloną
wartość; nie zmienia to zadanej prędkości w komendzie misji.
Dla Mission Planner używaj eksportu `.waypoints` (domyślny format).
Naprawiono także metadane istniejących plików w `missions/`, zachowując ich
komendy i współrzędne. Import sprawdzono biblioteką z lokalnej instalacji
Mission Planner (`ReadFile` i `ConvertToLocationwps`), bez uruchamiania lotu.


### Osobny Delay dla pętli i riskmappingu

W `missions/suas_2026_config.json` są dostępne pola:

```json
"lap_delay_s": 0,
"riskmapping_delay_s": 3
```

To przykład pętli bez postoju i 3 sekund postoju w każdym punkcie zdjęć.
`lap_delay_s` dotyczy wszystkich waypointów pętli oraz waypointu zmiany
wysokości na jej końcu. `riskmapping_delay_s` dotyczy pierwszego punktu
riskmappingu, kolejnych punktów zdjęć i home przed lądowaniem.
Pominięte pole przyjmuje 3 sekundy. W dołączonym JSON obie wartości początkowo
wynoszą 3; ustaw 0, aby wyłączyć postój w wybranym etapie.

Dla Mission Planner wartości ułamkowe są zaokrąglane w górę do pełnych sekund,
a dopuszczalny zakres wynosi 0–65535. `--settle-seconds` ma pierwszeństwo
przed JSON i ustawia obie wartości jednocześnie. Raport zawiera oba opóźnienia
oraz ich sumę (`total_delay_seconds`), uwzględnioną w czasie misji.
Po edycji JSON ponownie uruchom `python3 generate_suas_mission.py` i wczytaj
nowy plik `suas_2026.waypoints`; zapisany wcześniej plik testowy nie zmienia się
przez samą edycję konfiguracji.

### Wybór Search Boundary przez komentarze

Na początku `missions/suas_2026_config.json` znajdują się dwa bloki
`area_points`: Search Boundary 1 jest aktywny, a Search Boundary 2 otacza
komentarz `/* ... */`. Aby użyć drugiego obszaru, otocz cały pierwszy blok
(wraz z przecinkiem po tablicy) komentarzem `/* ... */`, a z drugiego usuń
zewnętrzne znaczniki komentarza. Zawsze pozostaw dokładnie jeden aktywny blok.
Generator odrzuca duplikaty zamiast arbitralnie wybierać jeden obszar.

Konfiguracja używa teraz JSON z komentarzami (JSONC), mimo zachowania nazwy
`.json`. Generator obsługuje `//` i `/* ... */`; pliki wynikowe do Mission
Planner zachowują dotychczasowy format. Punkty pętli pozostają demonstracyjne
poza nową granicą i wymagają uzupełnienia przed generowaniem misji.
