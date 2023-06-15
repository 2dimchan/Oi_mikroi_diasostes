ΟΜΑΔΑ : ΜΙΚΡΟΙ ΔΙΑΣΩΣΤΕΣ
Github: https://github.com/2dimchan/Oi_mikroi_diasostes
Website:  https://openedtech.ellak.gr/robotics2023/mikri-diasostes-2o-dimotiko-scholio-chanion/

Η Ελλάδα είναι μία χώρα με μεγάλη σεισμική δραστηριότητα. Η πρόβλεψη και η αντιμετώπιση των σεισμών, είναι ένα από τα μείζονα  θέματα της πολιτείας, αφού σχεδόν καθημερινά βιώνουμε σεισμούς μικρής ή μεγάλης κλίμακας. Θέλοντας να δώσουμε λύση σε καταστάσεις έκτακτης ανάγκης που δημιουργούνται μετά από ένα καταστροφικό σεισμό, σκεφτήκαμε να δημιουργήσαμε ένα σύστημα διαχείρισης εκτάκτων αναγκών της Πολιτικής Προστασίας. 
Συζητώντας με τα παιδιά, κάνοντας διεξοδική έρευνα και τέλος  προσκαλώντας  τον υπεύθυνο Πολιτικής Προστασίας κ. Βερικοκίδη Ι.  στο σχολείο μας, δημιουργήσαμε το εξής σχέδιο δράσης:
Μετά από ένα σεισμό μεγάλης κλίμακας η ομάδα διαχείρισης κρίσεων και καταστροφών της Πολιτικής Προστασίας θα πρέπει να αντιμετωπίσει τις εξής καταστάσεις:
	Αναγνώριση παλιρροιακού κύματος (τσουνάμι)
	Αναγνώριση μετασεισμικών δονήσεων
	Αναγνώριση καιρικών συνθηκών
	Αναγνώριση διαρροών φυσικού αερίου
	Αναγνώριση σεισμών
	Έλεγχος ηλεκτρικών εγκαταστάσεων
	Ειδοποίηση Πυροσβεστικής
	Διακοπή παροχής φυσικού αερίου και ηλεκτρισμού
	Οπτική και ακουστική ειδοποίηση πολιτών για προσέλευση στα σημεία συγκέντρωσης 
	Εντοπισμός εγκλωβισμένων στα ερείπια

Υλοποίηση Σεναρίου

Το έργο μας αποτελείται από 
•	Αισθητήρα στάθμης υγρών (νερού) για αναγνώριση παλιρροιακού κύματος
•	Αισθητήρας αναγνώρισης αερίων (διαρροή φυσικού αερίου)
•	Πολλαπλός ισθητήρας υπερύθρων (αναγνώριση πυρκαγιάς)
•	Αισθητήρας επιτάχυνσης (εντοπισμός σεισμικών δονήσεων)
•	Αισθητήρας θερμοκρασίας, υγρασίας περιβάλλοντος
•	Αισθητήρας βροχής
•	Ηλεκτρονόμοι για τη διακοπή παροχής ηλεκτρικού και φυσικού αερίου καθώς και ειδοποίηση πυροσβεστικής
•	Οθόνη LCD και buzzerγια την ειδοποίηση των πολιτών
•	  arduino mega = ελέγχει και διαχειρίζεται όλους τους προαναφερόμενους αισθητήρες και ηλεκτρονόμους
•	Για τον εντοπισμό των εγκλωβισμένων η Πολιτική Προστασία έχει προνοήσει και έχει διανέμει σε όλους τους πολίτες συσκευές εντοπισμού. Αυτές ενεργοποιούνται από τους πολίτες που βρίσκονται κάτω από τα ερείπια διευκολύνοντας το έργο των διασωστών οι οποίοι εντοπίζουν με ακρίβεια την τοποθεσία των πολιτών που κινδυνεύουν. Η συσκευή αποτελείται από έναν πομπό 433 MHZ και ένα arduino καθώς και από ένα δέκτη και ένα arduino nano και μία LCD οθόνη.

ΟΙ ΜΑΘΗΤΕΣ
ΚΑΛΟΓΕΡΟΠΟΥΛΟΣ ΑΛΕΞΑΝΔΡΟΣ
ΚΑΣΤΡΙΝΑΚΗς ΜΑΝΟΛΗΣ
ΚΑΡΑΓΚΟΥΝΗ ΑΝΑΣΤΑΣΙΑ
ΕΛ ΜΕΛΙΓΚΙ ΜΑΛΑΚ
ΝΙΝΟ ΟΛΓΑ
ΞΕΝΙΚΑΚΗΣ ΗΛΙΑΣ
ΦΟΥΝΤΟΥΛΑΚΗΣ ΒΑΓΓΕΛΗΣ
ΠΑΤΕΡΟΜΙΧΕΛΑΚΗΣ ΚΩΝ/ΝΟΣ
ΓΕΡΑΚΗ ΑΛΕΞΑΝΔΡΑ
ΔΑΝΑΗ
ΛΟΥΒΕΡΔΗΣ ΣΤΕΦΑΝΟΣ
ΣΠΑΝΟΥΔΑΚΗ ΝΕΦΕΛΗ 
ΠΑΠΑΔΟΜΑΝΩΛΑΚΗ ΜΕΛΙΝΑ 





Έργο
Για την δημιουργία του έργου προηγήθηκαν κάποια βασικά βήματα. 
Κατανόηση του Arduino και των βασικών λειτουργιών https://docs.arduino.cc/learn/starting-guide/getting-started-arduino 
https://www.vodafonegenerationnext.gr/lessons/gnwrimia-me-to-arduino-odhgos-gia-ton-ekpaideytiko 
Κατανόηση των διαφόρων εξαρτημάτων 
Τα παιδία χωρίστηκαν σε ομάδες. Κάθε ομάδα ανέλαβε να ψάξει στο διαδίκτυο πως λειτουργεί το κάθε εξάρτημα και να το παρουσιάσει στα υπόλοιπα παιδία.
Gyro
https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
LCD i2c
https://lastminuteengineers.com/i2c-lcd-arduino-tutorial/
Gas Sensor
https://circuitdigest.com/microcontroller-projects/interfacing-mq2-gas-sensor-with-arduino
https://lastminuteengineers.com/mq2-gas-senser-arduino-tutorial/
Rain Sensor
https://lastminuteengineers.com/rain-sensor-arduino-tutorial/
temperature and humidity sensor
https://lastminuteengineers.com/dht11-module-arduino-tutorial/
Flame Sensor
https://projecthub.arduino.cc/SURYATEJA/arduino-modules-flame-sensor-e48e97
Relay 4 module
https://lastminuteengineers.com/two-channel-relay-module-arduino-tutorial/
buzzer module
https://arduinointro.com/projects/adding-sounds-to-arduino-using-the-mh-fmd-piezo-buzzer-module 
 https://www.circuitbasics.com/how-to-use-active-and-passive-buzzers-on-the-arduino/ 
Tsounami module
https://arduinogetstarted.com/tutorials/arduino-switch 
Interupts
https://reference.arduino.cc/reference/tr/language/functions/external-interrupts/attachinterrupt/ 
RF 433MHz Transmitter/Receiver 

https://randomnerdtutorials.com/rf-433mhz-transmitter-receiver-module-with-arduino/ 

Στην συνέχεια τα παιδιά ενσωμάτωσαν των κώδικά τους σε ένα ενιαίο πρόγραμμα χρησιμοποιώντας functions για κάθε αισθητήρα. Έτσι παρακολούθησαν πως μπορούν όλα τα εξαρτήματα να συνεργαστούν μαζί. Καθώς και τα προβλήματα που παρουσιάστηκαν. Πχ η χρήση της εντολής delay σε αρκετά σημεία του προγράμματος  έκανε όλη την διαδικασία να καθυστερεί σημαντικά.
Το επόμενο βήμα ήταν η ο συνδυασμός αισθητήρων, οθόνης LCD, buzzer και ηλεκτρονόμων. 
Υλικά για το κέντρο επιχειρήσεων 

1 Funduino Mega2560 Rev3 24,11€
1 Basic 20x4 Character LCD 7,98€ 
1 Passive Buzzer Module 1,2€
1 Relay Module - 4 Channel 5V 4,76€
1 Waveshare Αισθητήρας Προπανίου - MQ-2  5,56€
1 Αισθητήρας Βροχής 1,45€
1 Αισθητήρας Στάθμης Υγρών 2,26€
1 Αισθητήρας Υγρασίας & Θερμοκρασίας DHT11 1,94€
1 Αισθητήρας Φωτιάς - 5-Channels 7,18€
1 Επιταχυνσιόμετρο & Γυροσκόπιο 3 Αξόνων IMU - MPU6050 3,87€
1 Tact Switch 12x12mm 7.3mm 0,15€

Σύνολο 60,46€
Υλικά για τους διασώστες
1 Nano Compatible - CH340  6,8€
1  DFRobot Beetle BLE - Based on Arduino Uno with Bluetooth 4.0   15.00€
1 RF Link Transmitter and Receiver - 433MHz 3,06€
1 Basic 16x2 Character LCD - White on Blue 5V  3,9€
1 Αντίσταση Carbon 1/4W 5% 220ohm 0,02€
1 Trimmer 6mm Single Turn - 2Kohm (Horizontal) 0,15€
1 Tact Switch 12x12mm 7.3mm 0,15€
Σύνολο 29,08€
Γενικό σύνολο 89,54€



 

Κύκλωμα 
Κέντρο επιχειρήσεων Πολιτικής Προστασίας
  
Διασώστες 
 
Pins
Arduino Mega
A0 Flame Sensor Analog port input 
A1 Flame Sensor Analog port input
A2 Flame Sensor Analog port input
A3 Flame Sensor Analog port input
A4 Flame Sensor Analog port input 
A6 Rain Sensor input
A7 GAS Sensor input 
D2 Interrupt Port Push Button Input
D3 Tsounami Sensor Digital Port 
D6 temperature Sensor Imput 
D7 GAS Sensor* Digital Pin  input 
D8  Buzzer Digital port 8 output
D39 Relay Digital Pin 39 output
D37 Relay Digital Pin 37 output
D35 Relay Digital Pin 35 output
D33 Relay Digital Pin 33 output

Arduino nano 
D4 Received module Digital Port input
D6 LCD module output
D7 LCD module output
D8 LCD module output
D9 LCD module output
D11 LCD module output

D12 LCD module 

Dfrobot beetle
D5 Transmit module Digital Port output


Βρισκόμαστε στο Κέντρο Επιχειρήσεων Πολιτικής Προστασίας  μετά από ένα μεγάλο σεισμό.
Βασικός σκοπός είναι η ενημέρωση πολιτών καθώς και η προστασία τους.
Με τις οθόνες LCD που υπάρχουν σε διάφορα σημεία στην πόλη δίνετε ενημερωτικές πληροφορίες .Υπάρχουν σειρήνες που ενεργοποιούνται σε περιπτώσεις ανάγκης. Υπάρχει γραμμή ενεργοποίησης Πυροσβεστικής , ΕΚΑΒ
Σενάριο 1
Αν ο αισθητήρας στάθμης υγρών(φλοτερ)  ενεργοποιηθεί σημαίνει  ότι  υπάρχει πιθανότητα για τσουναμι (Η θάλασσα έχει τραβηχτεί από τις παραλίες) Τότε γίνετε ενημέρωση των πολιτών μέσω LCD να μεταβούν σε ψηλότερο σημείο καθώς και ενεργοποιείτε  σειρήνα.
Σενάριο 2
Αν στην πόλη εμφανιστεί πυρκαγιά θα ενεργοποιηθεί ένας από τους 5 υπέρυθρους αισθητήρες τότε θα ειδοποιηθεί η Πυροσβεστική και θα εμφανιστεί στην οθόνη LCD η περιοχή που υπάρχει φωτία.
Σενάριο 3
Αν γίνει σεισμός αρκετά μεγάλος που γίνετε αντιληπτός από τον αισθητήρα επιτάχυνσης τότε εμφανίζεται μήνυμα στην οθόνη LCD καθώς και κλείνουν το φυσικό αέριο και το ηλεκτρικό ρεύμα μέσω των Relays και ενεργοποιείτε  σειρήνα.
Σενάριο 4
Αν βρέξει ή αν υπάρχει πτώση θερμοκρασία τότε υπάρχει ειδοποίηση μέσω των LCD να παραλάβουν σκηνές.
Σενάριο 5
Αν υπάρχει διαρροή Φυσικού αερίου τότε ενεργοποιείτε η διακοπή παροχής φυσικού αερίου καθώς και  ηλεκτρικής ενέργειας και ενεργοποιείτε  σειρήνα..   
 
