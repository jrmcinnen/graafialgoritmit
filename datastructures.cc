// Datastructures.cc

#include "datastructures.hh"

#include <random>

#include <cmath>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{

}

Datastructures::~Datastructures()
{
    clear_ways();
    clear_all();
}

int Datastructures::place_count()
{
    return places_.size();
}

void Datastructures::clear_all()
{
    places_.clear();
    areas_.clear();
    places_alphabetically_.clear();
    places_by_coords_.clear();
}

std::vector<PlaceID> Datastructures::all_places()
{
    std::vector<PlaceID> all_places = {};
    for(auto& place: places_){
        all_places.push_back(place.first);
    }
    return all_places;
}

bool Datastructures::add_place(PlaceID id, const Name& name, PlaceType type, Coord xy)
{
    int old_place_count = place_count();
    std::shared_ptr<Place> new_place = std::make_shared<Place>();
    new_place->id = id;
    new_place->name = name;
    new_place->type = type;
    new_place->location = xy;
    places_.insert(std::make_pair(id, new_place));
    if(old_place_count == place_count()){
        return false;
    }
    places_alphabetically_.insert(std::make_pair(name, new_place));
    places_by_coords_.insert(std::make_pair(xy, new_place));
    return true;
}

std::pair<Name, PlaceType> Datastructures::get_place_name_type(PlaceID id)
{
    auto it = places_.find(id);
    if(it != places_.end()){
        Name name = it->second->name;
        PlaceType type = it->second->type;
        return {name, type};
    }
    return {NO_NAME, PlaceType::NO_TYPE};
}

Coord Datastructures::get_place_coord(PlaceID id)
{
    auto it = places_.find(id);
    if( it != places_.end() ){
        return it->second->location;
    }
    return NO_COORD;
}

bool Datastructures::add_area(AreaID id, const Name &name, std::vector<Coord> coords)
{
    unsigned int old_area_count = areas_.size();
    std::shared_ptr<Region> new_area = std::make_shared<Region>();
    new_area->id = id;
    new_area->name = name;
    new_area->shape = coords;
    areas_.insert(std::make_pair(id, new_area));
    if(areas_.size() == old_area_count){
        return false;
    }
    return true;
}

Name Datastructures::get_area_name(AreaID id)
{
    if(areas_.find(id) != areas_.end()){
        return areas_.at(id)->name;
    }
    return NO_NAME;
}

std::vector<Coord> Datastructures::get_area_coords(AreaID id)
{
    if(areas_.find(id) != areas_.end()){
        return areas_.at(id)->shape;
    }
    return {NO_COORD};
}

void Datastructures::creation_finished()
{

}


std::vector<PlaceID> Datastructures::places_alphabetically()
{
    std::vector<PlaceID> places_in_order = {};
    for(auto& place: places_alphabetically_){
        places_in_order.push_back(place.second->id);
    }
    return places_in_order;
}

std::vector<PlaceID> Datastructures::places_coord_order()
{
    std::vector<PlaceID> places_in_order = {};
    for(auto& place: places_by_coords_){
        places_in_order.push_back(place.second->id);
    }
    return places_in_order;
}

std::vector<PlaceID> Datastructures::find_places_name(Name const& name)
{
    std::vector<PlaceID> places_with_right_name = {};
    auto it_pair = places_alphabetically_.equal_range(name);
    for(auto it = it_pair.first; it != it_pair.second; ++it){
        places_with_right_name.push_back(it->second->id);
    }
    return places_with_right_name;
}

std::vector<PlaceID> Datastructures::find_places_type(PlaceType type)
{
    std::vector<PlaceID> places_with_right_type = {};
    for(auto& place:places_){
        if(place.second->type == type){
            places_with_right_type.push_back(place.first);
        }
    }
    return places_with_right_type;
}

bool Datastructures::change_place_name(PlaceID id, const Name& newname)
{
    if(places_.find(id) != places_.end()){
        Name old_name = places_.at(id)->name;
        places_.at(id)->name = newname;
        auto it_pair = places_alphabetically_.equal_range(old_name);
        for(auto it = it_pair.first; it != it_pair.second; ++it){
            if(it->second->id == id){
                places_alphabetically_.insert(std::make_pair(newname,
                                                             it->second));
                places_alphabetically_.erase(it);
                break;
            }
        }
        return true;
    }
    return false;
}

bool Datastructures::change_place_coord(PlaceID id, Coord newcoord)
{
    if(places_.find(id) != places_.end()){
        Coord old_location = places_.at(id)->location;
        places_.at(id)->location = newcoord;
        auto it_pair = places_by_coords_.equal_range(old_location);
        for(auto it = it_pair.first; it != it_pair.second; ++it){
            if(it->second->id == id){
                places_by_coords_.insert(std::make_pair(newcoord,it->second));
                places_by_coords_.erase(it);
                break;
            }
        }
        return true;
    }
    return false;
}

std::vector<AreaID> Datastructures::all_areas()
{
    std::vector<AreaID> all_areas ={};
    for(auto& area: areas_){
        all_areas.push_back(area.first);
    }
    return all_areas;
}

bool Datastructures::add_subarea_to_area(AreaID id, AreaID parentid)
{
    auto it_parent = areas_.find(parentid);
    auto it_child = areas_.find(id);
    if(it_child == areas_.end() or it_parent == areas_.end()){
        return false;
    } else if(it_child->second->parent_area != nullptr){
        return false;
    }
    it_child->second->parent_area = it_parent->second;
    it_parent->second->subareas.push_back(it_child->second);
    return true;
}

std::vector<AreaID> Datastructures::subarea_in_areas(AreaID id)
{
    std::vector<AreaID> parent_areas = {};
    auto it = areas_.find(id);
    if(it == areas_.end()){
        return {NO_AREA};
    }
    std::shared_ptr<Region> parent = it->second->parent_area;
    while(parent != nullptr){
        parent_areas.push_back(parent->id);
        parent = parent->parent_area;
        }
    return parent_areas;
}

std::vector<PlaceID> Datastructures::places_closest_to(Coord xy, PlaceType type)
{
    std::vector<PlaceID> close_places_vector = {};
    std::map<float,PlaceID> close_places_map = {};
    for(auto& place: places_){
       if(type == PlaceType::NO_TYPE or place.second->type == type){
           float distance = xy-place.second->location;
           if(close_places_map.size() >= 3 and
                   distance < close_places_map.rbegin()->first){
               close_places_map.erase(close_places_map.rbegin()->first);
               close_places_map.insert(std::make_pair(distance, place.first));
           } else if(close_places_map.size() < 3) {
               close_places_map.insert(std::make_pair(distance, place.first));
           }
       }
    }
    for(auto it = close_places_map.begin(); it != close_places_map.end(); ++it){
        close_places_vector.push_back(it->second);
    }
    return close_places_vector;
}

bool Datastructures::remove_place(PlaceID id)
{
    if(places_.find(id) == places_.end()){
        return false;
    }
    Name name_to_erase = places_.at(id)->name;
    Coord coord_to_erase = places_.at(id)->location;
    auto it_pair_name = places_alphabetically_.equal_range(name_to_erase);
    while(it_pair_name.first != it_pair_name.second
          or it_pair_name.first != places_alphabetically_.end()){
        if(it_pair_name.first->second->id == id){
            places_alphabetically_.erase(it_pair_name.first);
            break;
        }
        ++it_pair_name.first;
    }
    auto it_pair_coords = places_by_coords_.equal_range(coord_to_erase);
    while(it_pair_coords.first != it_pair_coords.second
          or it_pair_coords.first == places_by_coords_.end()){
        if(it_pair_coords.first->second->id == id){
            places_by_coords_.erase(it_pair_coords.first);
            break;
        }
        ++it_pair_coords.first;
    }
    places_.erase(id);
    return true;
}

std::vector<AreaID> Datastructures::all_subareas_in_area(AreaID id)
{
    std::vector<AreaID> all_subareas = {};
    auto it = areas_.find(id);
    if(it == areas_.end()){
        return {NO_AREA};
    }
    get_subareas(it->second, all_subareas);
    return all_subareas;
}

AreaID Datastructures::common_area_of_subareas(AreaID id1, AreaID id2)
{
    if(areas_.find(id1) == areas_.end() or areas_.find(id2) == areas_.end()){
        return NO_AREA;
    }
    std::vector<AreaID> parent_areas1 = subarea_in_areas(id1);
    std::vector<AreaID> parent_areas2 = subarea_in_areas(id2);

    if(parent_areas1.size() == 0 or parent_areas2.size() == 0){
        return NO_AREA;
    }
    for(unsigned long int i = 0; i < parent_areas1.size(); ++i){
        for(unsigned long int j = 0; j < parent_areas2.size(); ++j){
            if(parent_areas1.at(i) == parent_areas2.at(j)){
                return parent_areas1.at(i);
            }
        }
    }
    return NO_AREA;
}

void Datastructures::get_subareas(const std::shared_ptr<Region>& area,
                                  std::vector<AreaID>& subareas)
{
    if(area->subareas.size() != 0){
        for(auto& area: area->subareas){
            subareas.push_back(area->id);
            get_subareas(area, subareas);
        }
    }
}

std::vector<WayID> Datastructures::all_ways()
{
    std::vector<WayID> all_ways = {};
    for(auto& way: ways_){
        all_ways.push_back(way.first);
    }
    return all_ways;
}

bool Datastructures::add_way(WayID id, std::vector<Coord> coords)
{   
    if(ways_.find(id) != ways_.end() ){
        return false;
    }
    Way* new_way = new Way;
    new_way->id = id;
    new_way->coords = coords;
    Distance way_length = 0;
    for(unsigned long int i = 0; i < coords.size()-1; ++i){
       way_length += floor(coords.at(i)-coords.at(i+1));
    }
    new_way->length = way_length;
    ways_.insert(std::make_pair(id, new_way));
    Coord start = coords.front();
    Coord end = coords.back();
    if(crossroads_.find(start) == crossroads_.end()){
        Crossroad* new_crossroad = new Crossroad;
        new_crossroad->location = start;
        crossroads_.insert(std::make_pair(start, new_crossroad));
    }
    if(crossroads_.find(end) == crossroads_.end()){
        Crossroad* new_crossroad = new Crossroad;
        new_crossroad->location = end;
        crossroads_.insert(std::make_pair(end, new_crossroad));
    }
    crossroads_.at(start)->neighbours.insert(std::make_pair(id, crossroads_.at(end)));
    crossroads_.at(end)->neighbours.insert(std::make_pair(id, crossroads_.at(start)));
    return true;
}

std::vector<std::pair<WayID, Coord>> Datastructures::ways_from(Coord xy)
{
    if( crossroads_.find(xy) == crossroads_.end() ){
        return {};
    }
    std::vector<std::pair<WayID, Coord>> all_ways_from = {};
    Crossroad* from = crossroads_.at(xy);
    for(auto& way: from->neighbours){
        all_ways_from.push_back(std::make_pair(way.first, way.second->location));
    }
    return all_ways_from;
}

std::vector<Coord> Datastructures::get_way_coords(WayID id)
{
    auto it = ways_.find(id);
    if(it != ways_.end() ){
        return it->second->coords;
    }
    return {NO_COORD};
}

void Datastructures::clear_ways()
{
    for(auto& way: ways_){
        delete way.second;
    }
    for(auto& crossroad: crossroads_){
        delete crossroad.second;
    }
    ways_.clear();
    crossroads_.clear();
}

std::vector<std::tuple<Coord, WayID, Distance> > Datastructures::route_any(Coord fromxy, Coord toxy)
{
    return route_least_crossroads(fromxy, toxy);
}

bool Datastructures::remove_way(WayID id)
{
    if(ways_.find(id) == ways_.end()){
        return false;
    }
    Way* removed_way = ways_.at(id);
    Crossroad* start = crossroads_.at(removed_way->coords.front());
    Crossroad* end = crossroads_.at(removed_way->coords.back());
    start->neighbours.erase(id);
    end->neighbours.erase(id);
    if(start->neighbours.size() == 0){
        delete start;
        crossroads_.erase(removed_way->coords.front());
    }
    if(end->neighbours.size() == 0){
        delete end;
        crossroads_.erase(removed_way->coords.back());
    }
    delete removed_way;
    ways_.erase(id);
    return true;
}

std::vector<std::tuple<Coord, WayID, Distance> > Datastructures::route_least_crossroads(Coord fromxy, Coord toxy)
{
    initialize_crossroads();
    if(crossroads_.find(fromxy) == crossroads_.end() or
            crossroads_.find(toxy) == crossroads_.end()){
        return {{NO_COORD, NO_WAY, NO_DISTANCE}};
    } else if(fromxy == toxy){
        return {{toxy, NO_WAY, 0}};
    }
    std::vector<std::tuple<Coord, WayID, Distance>> route = {};
    std::queue<Crossroad*> q = {};
    Crossroad* s = crossroads_.at(fromxy);
    s->color = 1;
    s->d = 0;
    crossroads_initialized_ = false;
    q.push(s);
    while(not q.empty()){
        Crossroad* u = q.front();
        q.pop();
        for(auto& crossroad: u->neighbours){
            Crossroad* v = crossroad.second;
            if(v->color == 0){
                v->color = 1;
                v->d = u->d + ways_.at(crossroad.first)->length;
                v->from = crossroad.first;
                q.push(v);
                if(v->location == toxy){
                    route.push_back(std::make_tuple(v->location, NO_WAY, v->d));
                    Crossroad* previous = v;
                    Crossroad* next = v->neighbours.at(v->from);
                    while(next->from !=  NO_WAY){
                        route.push_back(std::make_tuple(next->location, previous->from, next->d));
                        next = next->neighbours.at(next->from);
                        previous = previous->neighbours.at(previous->from);
                    }
                    route.push_back(std::make_tuple(next->location, previous->from, next->d));
                    std::reverse(route.begin(), route.end());
                    return route;
                }
            }
        }
        u->color = 2;
    }
    return route;
}

std::vector<std::tuple<Coord, WayID> > Datastructures::route_with_cycle(Coord fromxy)
{
    initialize_crossroads();
    if(crossroads_.find(fromxy) == crossroads_.end()){
        return {{NO_COORD, NO_WAY}};
    }
    crossroads_initialized_ = false;
    std::vector<std::tuple<Coord, WayID>> route = {};
    std::stack<Crossroad*> s = {};
    s.push(crossroads_.at(fromxy));
    Crossroad* u = nullptr;
    while(not s.empty()){
        u = s.top();
        s.pop();
        if(u->color == 0){
            u->color = 1;
            s.push(u);
            for(auto& crossroad: u->neighbours){
                Crossroad* v = crossroad.second;
                if(v->color == 0){
                    v->from = crossroad.first;
                    s.push(v);
                } else if(v->color == 1 and  u->from != crossroad.first){
                    route.push_back(std::make_tuple(v->location, NO_WAY));
                    Crossroad* next = v->neighbours.at(crossroad.first);
                    route.push_back(std::make_tuple(next->location, crossroad.first));
                    Crossroad* previous = nullptr;
                    while(next->from !=  NO_WAY){
                        previous = next;
                        next = next->neighbours.at(next->from);
                        route.push_back(std::make_tuple(next->location, previous->from));
                    }
                    std::reverse(route.begin(), route.end());
                    return route;
                }
            }
        }
        else {
            u->color = 2;
        }
    }
    return route;
}

std::vector<std::tuple<Coord, WayID, Distance> > Datastructures::route_shortest_distance(Coord fromxy, Coord toxy)
{
    initialize_crossroads();
    if(crossroads_.find(fromxy) == crossroads_.end() or
            crossroads_.find(toxy) == crossroads_.end()){
        return {{NO_COORD, NO_WAY, NO_DISTANCE}};
    }
    if(fromxy == toxy){
        return {{toxy, NO_WAY, 0}};
    }
    std::vector<std::tuple<Coord, WayID, Distance> > route = {};
    std::priority_queue<std::pair<Distance, Crossroad*>> p = {};
    Crossroad* s = crossroads_.at(fromxy);
    s->d = 0;
    crossroads_initialized_ = false;
    p.push(std::make_pair(s->d,s));
    Crossroad* u = nullptr;
    while(not p.empty()){
        u = p.top().second;
        p.pop();
        if(u->location == toxy){
            route.push_back(std::make_tuple(u->location, NO_WAY, -1*u->d));
            Crossroad* previous = u;
            Crossroad* next = u->neighbours.at(u->from);
            while(next->from !=  NO_WAY){
                route.push_back(std::make_tuple(next->location, previous->from, -1*next->d));
                previous = next;
                next = next->neighbours.at(next->from);
            }
            route.push_back(std::make_tuple(next->location, previous->from, next->d));
            std::reverse(route.begin(), route.end());
            return route;
        }
        Crossroad* v = nullptr;
        for(auto& crossroad: u->neighbours){
            v = crossroad.second;
            if(v->color == 0){
                v->color = 1;
                p.push(std::make_pair(v->d,v));
            }
            Distance new_way_length = ways_.at(crossroad.first)->length;
            if(v->d < u->d - new_way_length){
                v->d = u->d - new_way_length;
                v->from = crossroad.first;
                p.push(std::make_pair(v->d,v));
            }
        }
        u->color = 2;
    }
    return route;
}

Distance Datastructures::trim_ways()
{
    initialize_crossroads();
    std::unordered_set<WayID> trimmed_ways = {};
    std::set<Crossroad*> done_crossroads = {};
    std::priority_queue<std::pair<Distance, Crossroad*>> p = {};
    Crossroad* s = crossroads_.begin()->second;
    s->d = 0;
    crossroads_initialized_ = false;
    p.push(std::make_pair(s->d,s));
    Crossroad* u = nullptr;
    while(done_crossroads.size() < crossroads_.size()){
        if(p.empty()){
            for(auto& crossroad: crossroads_){
                if(crossroad.second->color == 0){
                    s = crossroad.second;
                    s->d = 0;
                    p.push(std::make_pair(s->d,s));
                    break;
                }
            }
        }
        else{
            u = p.top().second;
            p.pop();
            if(u->color != 2){
                done_crossroads.insert(u);
                trimmed_ways.insert(u->from);
                Crossroad* v = nullptr;
                for(auto& crossroad: u->neighbours){
                    v = crossroad.second;
                    if(v->color == 0){
                        v->color = 1;
                        p.push(std::make_pair(v->d,v));
                    }
                    Distance new_way_length = ways_.at(crossroad.first)->length;
                    if(v->d < -new_way_length){
                        v->d = -new_way_length;
                        v->from = crossroad.first;
                        p.push(std::make_pair(v->d,v));
                    }
                }
                u->color = 2;
            }
        }
    }
    std::unordered_set<WayID> deleted_ways = {};
    for(auto& way: ways_){
        if(trimmed_ways.find(way.first) == trimmed_ways.end()){
            deleted_ways.insert(way.first);
        }
    }
    for(auto& id: deleted_ways){
        remove_way(id);
    }
    Distance new_distance = 0;
    for(auto& way: ways_){
        new_distance += way.second->length;
    }
    return new_distance;
}


void Datastructures::initialize_crossroads()
{
    if(not crossroads_initialized_){
        for(auto& crossroad: crossroads_){
            crossroad.second->color = 0;
            crossroad.second->d = -INT_MAX;
            crossroad.second->from = NO_WAY;
        }
    }
}
