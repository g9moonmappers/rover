# Kalmanfilter 

Kalmanfilteret tar i bruk IMU, hjulhastigheter og UWB trilaterering for å finne et mer nøyaktig estimat på posisjon (x,y) og vinkel theta. Den kjører på en arudino mega og sammenarbeider med en jetson nano gjennom robot.py. Se rapport for mer informasjon.

## Forbedringer
- Tune støyparameterne Q og R for å få best mulig estimat på tilstander
- Adaptiv systemstøy for hastighet. Forandre støyparameteren for hastighet basert på terrenget roboten befinner seg i. Hvis roboten befinner seg i områder hvor den sklir mye (for eks. i en bakke) så øk usikkerheten for hastighet gitt av hjul. Hvis roboten er i område hvor den ikke sklir like mye (for eks. på flat bakke), senk denne usikkerheten.
- Legge til magnetometer for bedre opddatering av vinkel.
- 
