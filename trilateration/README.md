# UWB Trilaterering
2D innendørs posisjonering med 5x BU-04 UWB-moduler og en Arduino Mega.

## Maskinvare
- 4x BU-04-moduler som fungerer som ankere, hver er koblet opp med en Arduino Nano ESP32 og et 7,4V batteri
- 1x BU-04-modul som fungerer som tag, koblet til Arduino Mega
- 1x Arduino Mega som kjører selve trilatereringen

## Kabling
**Ankere (Arduino Nano ESP32):**
<img width="3508" height="2481" alt="trilaterering" src="https://github.com/user-attachments/assets/c0a5c831-9592-49cc-9fbc-031b45a2ac6e" />


**Tag (Arduino Mega):**
<img width="4961" height="3508" alt="BU04_til_pc" src="https://github.com/user-attachments/assets/a76333a0-a9f8-49fb-9537-dfa68b2090ec" />

## Konfigurering av ankere og tag
Flash hvert anker og tag [med BU-04_flashing-koden](BU04_FLASHING.ino):

Anker:
```cpp
configureBU04(0, 1, 1, 1); // anchor 0
configureBU04(1, 1, 1, 1); // anchor 1
configureBU04(2, 1, 1, 1); // anchor 2
configureBU04(3, 1, 1, 1); // anchor 3
```

Tag:
```cpp
configureBU04(0, 0, 1, 1);  // tag 0
```

## Fysisk plassering
Plasser ankerne i de fire hjørnene av rommet/containeren:

```
BS1 (0, y) -------- BS3 (x, y)
|                            |
|                            |
BS0 (0, 0) -------- BS2 (x, 0)
```

Mål x og y i meter relativt til BS0-hjørnet som fungerer som origo til koordinatsystemet og oppdater [ankerPosisjoner i rover read-koden](BU04_ROVER_READ.ino).

## Kalibrering
1. Flash [kalibreringskoden](BU04_CALIBRATION.ino) til Arduino
2. Legg en tommestokk flatt på et bord med minimum 1,5 m lengde
3. Plasser tag på starten av tommestokken pekende mot tommestokken
4. Plasser et anker 10 cm unna tag-en pekende mot tag-en
5. Send et valgfritt tegn i Serial Monitor og trykk enter
6. Flytt deretter ankeret 10 cm lenger unna tag-en
7. Gjenta steg 5-6 helt til ankeret ligger på 1,5m
8. Lim inn resultatene i et Google Sheets/Excel ark og utfør lineær regresjon
9. Utfør kalibreringen flere ganger helt til du er fornøyd med nøyaktigheten, for oss var dette tre ganger for hver anker
10. Oppdater `kalibreringStigning` og `kalibreringSkjæringspunkt` i [rover read-koden](BU04_ROVER_READ.ino) med stigningstallet og konstantleddet fra kalibreringen
11. Utfør kalibreringen på resten av ankrene 

## Visualisering
1. Flash og kjør [rover read-koden](BU04_ROVER_READ.ino) på Arduino mega meg tag-en
2. Lukk Arduino Serial Monitor
3. Åpne [visualiseringskoden](BU04_ROVER_READ.ino) i Processing IDE
4. Sett riktig COM-port og oppdater ankerposisjonene
5. Kjør koden 

## Dataformat fra trilatereringen
På Serial Monitor ved 115200 baud får man :

```
x,y,dist0,dist1,dist2,dist3
```

- x, y: tag-posisjon i meter
- dist0-3: kalibrert avstand til hver basestasjon i meter
