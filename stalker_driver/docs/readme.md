# stalker_driver
 
Biblioteka komunikacji pomiędzy płytką STM (będącą sterownikiem fizycznym platformy vc200) a komputerem z systemem linuks.
Całość została napisana w języku C++ bazując na projekcie:
[STalker](https://github.com/cezarymalek/STalker) oraz zmianach naniesionych przez [współautora](https://github.com/PanAndrew)

Wykorzystano bibliotekę boost w celu obsługi połączenia UDP dlatego jest on wymagana do poprawnej kompilacji
 
Nie jest wymagana biblioteka roscpp ani inna która wymagała by kompilacji lub instalacji składników Robot Operating System.
 
## Organizacja projektu
 
Projekt podzielono według modułów obsługujących poszczególne elementy platformy (żyroskop, timery, sterownik napędów itd).

Interfejsy komunikacji zostały podzielone na Upstream (napływające z STM'a) oraz Downstream (wysyłane do STM'a)
 
Obiekt klasy STInterfaceClientUDP oczekuje rejestracji danych napływających z mikrokontrolera poprzez użycie funkcji

```c++
void addExpectedDataType(std::shared_ptr<Interface::UpstreamDataType> iExpectedDataType);
```

klasa ```Interface::UpstreamDataType``` jest klasą bazową dla każdego interfejsu odczytu danych

klasa ```Interface::DownstreamDataType``` jest klasą bazową dla zapisu danych do mikrokontrolera 

Na podstawie dziedziczenia z tych dwóch klas budujemy interfejs komunikacji
Projekt został przystosowany do wykorzystania wielowątkowości.
 
Z racji że do obiektu STInterfaceClientUDP przekazujemy shared pointer mamy dostęp do danych z poziomu funkcji odczytującej napływające dane z stm oraz funkcji pobierającej te dane do systemu sterującego.
 
odbieranie :
 
funkcja ```void STInterface::STInterfaceClientUDP::doReceive()``` wywołuje wewnątrz funkcje przez wskaźnik ```void deserialize(const uint8_t* iDataStream, const int iDataSize)```. Wewnątrz funkcji `deserialize` użyta jest blokada mutexu przy każdej operacji zapisu danych do struktury aż do zakończenia funkcji. Podobnie przy pobieraniu danych muteks jest blokowany.
 
W celu skorzystania z biblioteki należy:
 
1. Utworzyc interfejs UDP STInterface::STInterfaceClientUDP
```c++
try {
    stClientPtr_ = std::make_shared<STInterface::STInterfaceClientUDP>(1115, "192.168.1.10", "7");
    connected_ = true;
} catch (const boost::exception &e) {
    std::string diag = diagnostic_information(e);
}
```
2. Zarejestrować wszystkie interfejsy odczytu danych
```c++
void addExpectedDataType(std::shared_ptr<Interface::UpstreamDataType> iExpectedDataType);
```
 
3. Utworzyć wątek z wykonywaniem funkcji `void STInterface::STInterfaceClientUDP::doReceive()`
4. Uruchomić wątek odczytu danych
 
Propozycja uruchomienia zaprezentowana jest w projekcie vc200_driver w klasie `VC200Driver`
 
 
