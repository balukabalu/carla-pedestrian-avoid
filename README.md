# carla-pedestrian-avoid

Statikus gyalogos az út közepén, egy kis "csalással" a rewardolás közben, imitation learning-es beütéssel.

Főbb tulajdonságok:
- 
- Minden 5. epizódban nem a neurális hálózat irányít, hanem egy előre leprogramozott gyalogoskikerülő
- Zaj frekvenciájának csökkentése az által, hogy csak minden 15. lépésben változik a zaj mértéke. Ezzel konzisztensebb viselkedést lehet elérni
- Zaj csak akkor adódik hozzá az akcióhoz, ha a gyalogos látótávolságban van
- Gyalogos észlelése 8 és 18 méteres távolság között

start_learning file:
- 
- Neurális háló beállítása
  - TD3, tehát 1 actor, 2 critic
- Carla indítása
- Epizód indítása
  - Kezdeti állapot feltöltése 0-kkal
- Lépések:
  - 15 lépésenként új zaj, csak akkor, ha a gyalogos látótávon belül van
  - Step az akciók alapján
  - bufferhez való hozzáadás
  - (esetleg a szimmetrikus állapotok hozzáadása, ebben a verzióban nem kell, mert csak balra kerüli ki az autó a gyalogost)
  - Tanulás, minden második lépésben a hálók frissítése
- Epizód befejezése
  - Minden 3. lépés után a háló elmentése külön mappába
  - Epizód főbb tulajdonságainak fájlba írása
  - Újraindítás
  
carla_env file:
- 
- spect_cam: 
  - a kameraállás beállítása (12 méterrel az autó felett, felülnézetből)
- start2: 
  - kliens beállítása, város betöltése, megjelenítési és számítási beállítások, spawn pont beállítása, gyalogos észlelési távolságának random beállíása
- kill:
  - actor-ok törlése
- collevent:
  - akkor hívódik meg, ha ütközést észlel a Carla. Ha a gyalogossal ütközik, igaz-ra állít egy változót (collosion_with_ped)
- setup_car:
  - alapból hamisra állítja a collosion_with_ped változót
  - beállítja a járművet (Audi E-Tron)
  - beállítja az ütközés szenzort
  - beállít néhány változót, ami fontos lesz (út szélessége, sáv ID-je, autó sebessége, gyalogos látható-e)
- setup_pedestrian:
  - megpróbálja inicializálni a gyalogost, és elindítani 
  - lerakja egy fix pontban
  - rendel hozzá egy sebességet úgy, hogy az autó elüsse
  - ez a rész enyhén bug-os a Carla-ban, ezért try-except-tel kellett megoldani, néha nem engedi elindítani a gyalogost
- braking_distance:
  - kiszámolja a gyalogos távolságának és az adott sebességnél való féktáv hányadosát
  - itt nem használjuk
- ped_sensor:
  - kiszámolja az autó és a gyalogos távolságát és szögét
  - az autó helyzetéhez hozzáad 2.5 métert, mert az autó közepét veszi alapból, nem pedig az elejét
  - ha 0 a távolság, akkor 0.001-et ad meg, hogy ne legyen probléma a 0-val való osztással
  - mert a távolság reciprokát adja meg (hogy amikor a gyalogos nem látszik, akkor lehessen 0-t adni a távolságértéknek)
- snap_to_s_t:
  - a world_snapshotot neurális háló számára elfogadható bemenetté alakítja
  - world snapshotból kinyeri az autó és a gyalogos snapshotját
  - számol sávhibát, szöghibát, x és y irányú sebességet, legyezési szögsebességet, x és y irányú gyorsulást, gyalogos távolságot és szöget
  - mivel a road_waypoint-nál az autó tartózkodási sávjának közepére vetíti az autót, ezért ha nem a saját sávjában van, akkor addig változtatja az y értéket, amíg az autó eredeti sávjába nem esik, és azt vetíti le sávközépre
  - ha ez nem lenne, akkor a szembejövő sáv közepénél is 0 sávhibát adna 
- automated_drive:
  - egy sima kontroller, ami az autót sávközépen tartja és sebességet tart
  - ha a brakes paraméter True, akkor az y irányú sebességet is próbálja egy bizonyos értéken tartani (hogy kanyarokban ne sodródjon ki)
  - ha az avoid paraméter True, akkor kikerüli a gyalogost úgy, hogy az ellenkező sáv közepére próbál tartani (statikus gyalogos esetén)
  - visszaadja a jármű irányítási értékeit
- throttlecontrol:
  - sebességtartó, tempomat
- step:
  - frissíti a amera helyzetét, snapshotot csinál, eldönti, hogy automatán vezessen-e a jármű, rewardot számol és eldönti, hogy vége legyen-e az epizódnak
  - minden 5. epizódnál automatán vezet
  - ha 10 méternél közelebb van a gyalogoshoz, akkor a másik sávba tart, de ha 8 méternél közelebb, visszatérne a saját sávjába (így tudja pont kikerülni a gyalogost) (csak automata üzemmódban)
  - beállítja a sebességfokozatot
  - rewardot számol
  - ugyanaz a reward, mint a TORCS-ban, csak minél közelebb van a gyalogoshoz, annál inkább kisebb szorzóval
  - ha nagyon közel van, akkor -1-hez közelít a szorzó, így a saját sávjában negatív jutalmat kap, azonban ha elkezd átmenni a másik sávba, akkor pozitívat
  - epizódok végén ha nekimegy valaminek, vagy nagy a sávhíba, vagy visszafordulna, akkor -10-et kap
  - ha kikerülte a gyalogost, és utána megy neki valaminek, akkor 10-et kap
  - ha elütötte, akkor -100 -at
  - ha kikerülte és nem ütközött, akkor 100*cos(szöghiba)*szigmoid(sávhiba)
  - a szigmoid függvény 1-et ad, ha a sávhiba 0, és 0-t ad, ha nagy
  - a "csalás": mivel tudom, hogy az út vektorának iránya (-1,0,0), és a gyalogos is fix helyen jelenik meg, csak az észlelési távolsága változik, ezért nem a gyalogoshoz viszonyítom az autó helyzetével kapcsolatos rewardokat, hanem az autó koordinátáit használom. 
  
  
eddigi eredmények:
- 
A jármű szépen, egyenesen halad, azonban felemás eredményeket hoz. Volt olyan még egy előző jutalomrendszerrel, hogy kikerülte a gyalogost, visszatért a sávjába, de nem kormányzott egyenesbe elég hamar, és nekiment egy oszlopnak. Volt olyan, hogy már sokkal korábban átment a szembejövő sávba, mint indokolt lett volna. Olyan is volt, hogy szépen kikerülte 7-800 epizód után, de utána ugyanúgy nekiment a gyalogosnak. 
Azért statikus még a gyalogos, mert elképzeléseim szeirnt ha ezt ki tudja kerülni, akkor valszeg a sétáló gyalogost is. Ugyanúgy, mint ahogy először egyenesen menni tanítottam meg a járművet, most statikus gyalogost kikerülni, aztán ha azt tudja, akkor sétáló gyalgost is.
A neurális hálób bemeneti és kimeneti dimenziójával nincsen probléma, mert azt szabadon tudom szerkeszteni. Egyenesen menni 7 dimenzióval tanult meg, utána a súlyokat magában foglaló .h5 fájban a táblázatba hozzáadtam még két oszlopot, és 10-20 epizódnyi tanulás után ugyanúgy ment egyenesen, kis megingás után. 

Tervek:
- 
A gyalogos távolsága és szöge helyett az autóhoz viszonyított x és y irányú koordinátái kellenek. Ugyanis jelen helyzetben ha lát egy gyalogost a szembesávban, akkor is a szembesávba próbál majd áttérni. Így a jutalmazást sem a gyalogoshoz mért távolsággal kell majd megoldani, hanem a gyalogos x és y irányú távolságával (nagy x irányú távolság --> y irányú távolság mindegy, csak maradjon sávközépen, kis x irányú távolság --> nagy y irányú távolság a kívánt, mert ki akarjuk kerülni. Ez a ma esti projekt.)


  
  
