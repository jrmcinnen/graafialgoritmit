// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <cmath>
#include <set>
#include <unordered_set>
#include <map>
#include <string>
#include <memory>
#include <vector>
#include <tuple>
#include <stack>
#include <utility>
#include <limits>
#include <queue>
#include <functional>
#include <QDebug>

// Types for IDs
using PlaceID = long long int;
using AreaID = long long int;
using Name = std::string;
using WayID = std::string;

// Return values for cases where required thing was not found
PlaceID const NO_PLACE = -1;
AreaID const NO_AREA = -1;
WayID const NO_WAY = "!!No way!!";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Enumeration for different place types
// !!Note since this is a C++11 "scoped enumeration", you'll have to refer to
// individual values as PlaceType::SHELTER etc.
enum class PlaceType { OTHER=0, FIREPIT, SHELTER, PARKING, PEAK, BAY, AREA, NO_TYPE };

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    float distance1 = sqrt(pow(c1.x,2)+pow(c1.y,2));
    float distance2 = sqrt(pow(c2.x,2)+pow(c2.y,2));
    if(distance1 != distance2){
        return distance1 < distance2;
    }
    return c1.y < c2.y;
}

inline float operator-(Coord c1, Coord c2)
{
    return sqrt(pow(c1.x-c2.x,2)+pow(c1.y-c2.y,2));
}



// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Duration is unknown
Distance const NO_DISTANCE = NO_VALUE;

struct Place
{
    PlaceID id = NO_PLACE;
    Name name = NO_NAME;
    PlaceType type = PlaceType::NO_TYPE;
    Coord location = NO_COORD;
};

struct Region
{
    AreaID id = NO_AREA;
    Name name = NO_NAME;
    std::vector<Coord> shape = {};
    std::shared_ptr<Region> parent_area = nullptr;
    std::vector<std::shared_ptr<Region>> subareas = {};
};

struct Way
{
  WayID id = NO_WAY;
  std::vector<Coord> coords = {};
  Distance length = 0;
};

struct Crossroad
{
  Coord location = NO_COORD;
  std::unordered_map<WayID, Crossroad*> neighbours = {};

  // Apumuutujat graafialgoritmeja varten
  int color = 0;
  int d = -INT_MAX;
  WayID from = NO_WAY;
};

inline bool operator<(Crossroad c1, Crossroad c2)
{
    return c1.d < c2.d;
}

// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: O(1)
    // Short rationale for estimate: Funkio käyttää std::size() algoritmia,
    // joka on vakioaikainen algoritmi.
    int place_count();

    // Estimate of performance: O(n+m), missä n on järjestelmän paikkojen määrä
    // ja m alueiden määrä.
    // Short rationale for estimate: Funktio kutsuu neljä kertaa std::clear()
    // algoritmia, jonka suoritusaika kasvaa lineaarisesti. Tätä kutsutaan kolmesti
    // paikkoja säilöviin rakenteisiin ja kerran alueita säilövään rakenteeseen.
    // (3*O(n)+O(m) -> O(n+m))
    void clear_all();

    // Estimate of performance: O(n), missä n on paikkojen lukumäärä
    // Short rationale for estimate: Funktio iteroi aina koko rakenteen läpi,
    // jossa on n alkioita. Jokaisella kierroksella kutsutaan std::push_back()
    // algoritmia, joka on vakioaikainen. (O(n)*vakio -> O(n))
    std::vector<PlaceID> all_places();

    // Estimate of performance: O(n), mutta keskimäärin Theta(log n),
    // missä n on paikkojen lukumäärä
    // Short rationale for estimate: Funktio kutsuu kerran std::insert()
    // algoritmia unordered_mapille, jolle pätee keskimäärin Theta(1), mutta
    // huonoimmassa tapauksessa O(n). Insertiä kutsutaan kahdesti multimapille,
    // jolle pätee O(log n). Kaikki muut funktion käyttämät algoritmit ovat
    // vakioaikaisia. (vakio+2*O(log(n)) -> Theta(log(n)), O(n)+2*O(log n) -> O(n))
    bool add_place(PlaceID id, Name const& name, PlaceType type, Coord xy);

    // Estimate of performance: O(n), mutta keskimäärin Theta(1), missä n on
    // paikkojen lukumäärä.
    // Short rationale for estimate: Funktio käyttää unordered mapille find()
    // algoritmia, jolle pätee Theta(1), mutta huonoimmassa tapauksessa O(n).
    std::pair<Name, PlaceType> get_place_name_type(PlaceID id);

    // Estimate of performance: O(n), mutta keskimäärin Theta(1), missä n on
    // paikkojen lukumäärä.
    // Short rationale for estimate: Funktio käyttää unordered mapille find()
    // algoritmia, jolle pätee Theta(1), mutta huonoimmassa tapauksessa O(n).
    Coord get_place_coord(PlaceID id);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O(n), missä n on paikkojen lukumäärä
    // Short rationale for estimate: Funktio iteroi aina koko rakenteen läpi,
    // jossa on n alkioita. Jokaisella kierroksella kutsutaan std::push_back()
    // algoritmia, joka on vakioaikainen. (O(n)*vakio -> O(n))
    std::vector<PlaceID> places_alphabetically();

    // Estimate of performance: O(n), missä n on paikkojen lukumäärä
    // Short rationale for estimate: Funktio iteroi aina koko rakenteen läpi,
    // jossa on n alkioita. Jokaisella kierroksella kutsutaan std::push_back()
    // algoritmia, joka on vakioaikainen. (O(n)*vakio -> O(n))
    std::vector<PlaceID> places_coord_order();

    // Estimate of performance: O(n), mutta Theta(log n), missä n on paikkojen
    // lukumäärä.
    // Short rationale for estimate: Alkion etsimiselle multimap rakenteesta
    // pätee O(log n). Vakioaikaista push_back algoritmia kutsutaan niin monta kertaa
    // kuin on haetun nimisiä alkio rakenteessa. Huonoimmassa tapauksessa niitä on
    // n kappaletta, eli pätee O(n). Mutta tämä tuskin on kovin yleistä. Voisi olettaa, että
    // suurin osan paikoista ei ole samannimisiä. Alkion hakeminen siis yleensä määrää
    // suoritusajan, joten pätee Theta(log n).
    std::vector<PlaceID> find_places_name(Name const& name);

    // Estimate of performance: O(n), missä n on paikkojen määrä.
    // Short rationale for estimate: Iteroidaan koko tietorakenteen läpi, jossa on
    // n alkioita. Jos paikan tyyppi pätee, niin kutsutaan push_back() algoritmia,
    // joka on vakioaikainen. Pahimmillaan siis O(n)*vakio -> O(n). Joka tapauksessa
    // on tutkittava kaikki n alkiota.
    std::vector<PlaceID> find_places_type(PlaceType type);

    // Estimate of performance: O(n), mutta keskimäärin Theta(log n), missä n on
    // paikkojen lukumäärä.
    // Short rationale for estimate: Funktio käyttää unordered mapille find()
    // algoritmia, jolle pätee Theta(1), mutta huonoimmassa tapauksessa O(n). Alkion
    // etsiminen multimap-rakenteesta etsimiselle puolestaan pätee O(log n).
    // Keskimäärin tämä multimap-rakenteen algoritmi määrää suoritusajan, eli Theta(log n).
    bool change_place_name(PlaceID id, Name const& newname);

    // Estimate of performance: O(n), mutta keskimäärin Theta(log n), missä n on
    // paikkojen lukumäärä.
    // Short rationale for estimate: Funktio käyttää unordered mapille find()
    // algoritmia, jolle pätee Theta(1), mutta huonoimmassa tapauksessa O(n). Alkion
    // etsiminen multimap-rakenteesta etsimiselle puolestaan pätee O(log n).
    // Keskimäärin tämä multimap-rakenteen algoritmi määrää suoritusajan, eli Theta(log n).
    bool change_place_coord(PlaceID id, Coord newcoord);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O(n), mutta keskimäärin Theta(1),
    // missä n on alueiden lukumäärä
    // Short rationale for estimate: Lisääminen unordered_map rakenteeseen on
    // Theta(1), mutta huonoimmassa tapauksessa O(n).
    bool add_area(AreaID id, Name const& name, std::vector<Coord> coords);

    // Estimate of performance: O(n), mutta keskimäärin Theta(1),
    // missä n on alueiden lukumäärä
    // Short rationale for estimate: Etsiminen unordered_map rakenteesta on
    // Theta(1), mutta huonoimmassa tapauksessa O(n).
    Name get_area_name(AreaID id);

    // Estimate of performance: O(n), mutta keskimäärin Theta(1),
    // missä n on alueiden lukumäärä
    // Short rationale for estimate: Etsiminen unordered_map rakenteesta on
    // Theta(1), mutta huonoimmassa tapauksessa O(n).
    std::vector<Coord> get_area_coords(AreaID id);

    // Estimate of performance: O(n), missä n on alueiden määrä
    // Short rationale for estimate: Iteroidaan rakenteen läpi, jossa on
    // n alkioita ja kutsutaan vakioaikaista push_back() -algoritmia.
    std::vector<AreaID> all_areas();

    // Estimate of performance: O(n), mutta keskimäärin Theta(1), missä
    // n on alueiden määrä.
    // Short rationale for estimate: Etsiminen unordered_mapista on yleensä
    // vakioaikaista, mutta huonoimmillaan O(n). Muuten käytetään vain
    // sijoitusoperaatiota ja vakioaikaista push_back() -algoritmia.
    bool add_subarea_to_area(AreaID id, AreaID parentid);

    // Estimate of performance: O(n), missä n on alueiden määrä
    // Short rationale for estimate: Pahimmillaan joudutaan käymään läpi
    // kaikki alueet (n kappaletta). Tämä on kuitenkin harvinainen tapaus.
    // Keskimääräistä tapausta on kuitenkin vaikea määrittää.
    std::vector<AreaID> subarea_in_areas(AreaID id);

    // Non-compulsory operations

    // Estimate of performance: O(1)
    // Short rationale for estimate: Tyhjä funktio.
    void creation_finished();

    // Estimate of performance: O(n), missä n on alueiden määrä
    // Short rationale for estimate: Käytetään luokan privaattia jäsen-
    // funktioita get_subareas(), jonka aikavaativuus on O(n).
    std::vector<AreaID> all_subareas_in_area(AreaID id);

    // Estimate of performance: O(n), missä n on paikkojen määrä
    // Short rationale for estimate: Iteroidaan koko rakenteen läpi, jossa
    // on n alkiota. Pahimmillaan jokaisella kierroksella lisätään ja poistetaan
    // alkiota map rakenteesta. Mapissa on maksimissaan kolme alkiota, joten nämä
    // operaatiot ovat oikeastaan vakioaikaisia, sillä  vaikka ne ovat yleisesti
    // O(n), mutta nyt n on rajoitettu.
    std::vector<PlaceID> places_closest_to(Coord xy, PlaceType type);

    // Estimate of performance: O(n), mutta keskimäärin Theta(log n), missä
    // n on paikkojen määrä.
    // Short rationale for estimate: Poistaminen unordered mapista on yleensä
    // vakioaikainen, mutta huonoimmilla O(n). Multimapista poistaminen
    // on O(log n). Metodi poistaa alkion multimapista kahdesti ja kerran unordered
    // mapista. Keskimäärin vakio+2*O(log n) -> Theta(log n),
    // Pahimmillaan O(n)+2*O(log n) -> O(n)
    bool remove_place(PlaceID id);

    // Estimate of performance: O(n), missä n on alueiden määrä
    // Short rationale for estimate: Hyödyntää funktioita subarea_in_areas() ja
    // suoritusaika kasvaa lineaarisesti eli O(n). Tämän jälkeen käydään kaksi
    // vektoria läpi sisäkkäisillä silmukoilla. Jos subarea_in_area funktiossa
    // totetui huonoin tapaus O(n), niin ensimmäinen yhteinen ylempi alue löytyy
    // käytännössä ensimmäisellä vertailulla. Keskimääräistä tapausta on vaikea
    // tässä määrittää.
    AreaID common_area_of_subareas(AreaID id1, AreaID id2);

    // Phase 2 operations

    // Jatkossa E = väylien lukumäärä
    // ja V = risteysten lukumäärä

    // Estimate of performance: O(E)
    // Short rationale for estimate: Iteroidaan kaikkien kulkuväylien läpi, joita
    // joita on siis E kappaletta.
    std::vector<WayID> all_ways();

    // Estimate of performance: O(E+log V), mutta keskimäärin Theta(log V).
    // Short rationale for estimate: Tehdään lisäyksiä rakenteisiin map ja
    // unordered_map. Mapiin lisäykselle pätee O(log V) ja unordered_mapille
    // O(E) ja Theta(1). Keskimäärin siis vakio+O(log V)->Theta(log V),
    // Pahimmillaan O(E)+O(log V)->O(E+log V)
    bool add_way(WayID id, std::vector<Coord> coords);

    // Estimate of performance: O(V), mutta keskimäärin Theta(log V).
    // Short rationale for estimate: Alkion löytäminen map-rakenteesta on O(log V).
    // Jos risteyksestä sattuisi olemaan yhteys kaikkii muihin risteyksiin,
    // jouduttaisiin, ne kaikki käymään läpi (V operaatiota).
    // Pahimmillaan O(log V)+O(V)->O(V). Keskimäärin yhdestä risteyksestä on
    // kuitenkin vain muutama yhteys, jolloin alkion etsiminen rajoitaa algoritmin
    // tehokkuutta. Pätee siis Theta(log V).
    std::vector<std::pair<WayID, Coord>> ways_from(Coord xy);

    // Estimate of performance: O(E), mutta keskimäärin Theta(1)
    // Short rationale for estimate: Etsitään alkiota unordered_map-rakenteesta.
    // Tälle operaatiolle pätee O(E), muuta keskimäärin Theta(1).
    std::vector<Coord> get_way_coords(WayID id);

    // Estimate of performance: O(E+V)
    // Short rationale for estimate: Poistetaan kaikki alkiot, eli kaikki väylät
    // ja risteykset käydään läpi.
    void clear_ways();

    // Estimate of performance: O(E+V)
    // Short rationale for estimate: Kutsuu route_least_crossroads() -metodia.
    std::vector<std::tuple<Coord, WayID, Distance>> route_any(Coord fromxy, Coord toxy);

    // Non-compulsory operations

    // Estimate of performance: O(E+log(V)), mutta keskimäärin Theta(log V)
    // Short rationale for estimate: Poistaminen unordered_mapista on yleensä
    // vakioaikainen, mutta huonoimmilla O(E). Mapista poistaminen
    // on O(log V). Keskimäärin vakio+O(log V) -> Theta(log V),
    // Pahimmillaan O(E)+O(log V) -> O(E+log V)
    bool remove_way(WayID id);

    // Estimate of performance: O(E+V)
    // Short rationale for estimate: Funktio noudattaa leveyteen ensin -hakua.
    // Leveyteen ensin -haun aikavaativuus on O(E+V).
    std::vector<std::tuple<Coord, WayID, Distance>> route_least_crossroads(Coord fromxy, Coord toxy);

    // Estimate of performance: O(E+V)
    // Short rationale for estimate: Funktio noudattaa syvyyteen ensin -hakua.
    // Syvyyteen ensin -haun aikavaativuus on O(E+V)
    std::vector<std::tuple<Coord, WayID>> route_with_cycle(Coord fromxy);

    // Estimate of performance: O((E+V)*log V)
    // Short rationale for estimate: Funktio noudattaa Dijkstran algoritmi.
    // Djikstran algoritmin aikavaativuus on O((E+V)*log V). Kertoimen log V
    // aiheuttaa prioriteettijonon pop()-metodi.
    std::vector<std::tuple<Coord, WayID, Distance>> route_shortest_distance(Coord fromxy, Coord toxy);

    // Estimate of performance: O((E+V)*log V+E)
    // Short rationale for estimate: Funktio noudattelee Primin algoritmia.
    // Toiminta on hyvin samanlainen kuin Djikstran algoritmissa. Prioriteettijono
    // aiheuttaa tässäkin kertoimen log V. Uutta tieverkkoa muodostettaessa käydään vielä
    // läpi kaikki E kappaletta väyliä, mikä aiheuttaa aikavaativuuteen lisän E.
    Distance trim_ways();

private:
    // Kerää kaikki annetun alueen alialueet.
    // Estimate of performance: O(n), n on alueiden määrä
    // Short rationale for estimate: Huonoimmassa tapauksessa joudutaan käymään
    // kaikki alueet läpi (n kappaletta). Näin käy jos ollaan puurakenteen juuressa.
    // Muissa tapauksissa nopeampi.
    void get_subareas(const std::shared_ptr<Region>& area,
                      std::vector<AreaID>& subareas);

    // Valmistelee risteykset reitin hakuja varten
    // Estimate of performance: O(V)
    // Short rationale for estimate: Alustetaan jokainen risteys.
    // Näitä on V kappaletta.
    void initialize_crossroads();

    std::unordered_map<PlaceID, std::shared_ptr<Place>> places_ = {};
    std::unordered_map<AreaID, std::shared_ptr<Region>> areas_ = {};

    std::multimap<Coord, std::shared_ptr<Place>> places_by_coords_ = {};
    std::multimap<Name, std::shared_ptr<Place>> places_alphabetically_ = {};

    std::map<Coord, Crossroad*> crossroads_ = {};
    std::unordered_map<WayID, Way*> ways_ = {};

    bool crossroads_initialized_ = true;
};

#endif // DATASTRUCTURES_HH
