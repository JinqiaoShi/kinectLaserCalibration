#ifndef MYMAP_H
#define MYMAP_H

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

using namespace std;
typedef pair<int, float> mapData;


typedef boost::unordered_map<int,mapData > hmap;
typedef hmap::iterator hitr;



class mymap : public hmap
{
public:
    //hmap map;
    void insert(std::pair<int,mapData> p)
    {
        hmap::iterator i = hmap::find(p.first);
        if(i==hmap::end()){
             hmap::insert(p);
        }
        else{
            mapData d=p.second;
            mapData actualVal=i->second;
            if(d.second<actualVal.second){
                actualVal.second=d.second;
            }
        }

    }
};

#endif
