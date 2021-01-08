#include<vector>
using namespace std;
float rplader_pole_distance(vector<float> vec);
template<typename T> 
void sort_and_deduplication(T &c)
{
    sort(c.begin(),c.end());
    auto new_end=unique(c.begin(),c.end());
    c.erase(new_end,c.end());
}