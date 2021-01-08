#include "PoleExtraction.hpp"
#include<map>
#include<cmath>
#include<vector>
#include<iostream>
#include<algorithm>
using namespace std;
float rplader_pole_distance(vector<float> vec_a)
{
    map<int,vector<float>> distance;
    int count=0;
    float pole_distance=0;
    vector<int> key_vec;
    for(auto i:vec_a)//put all vector data in map,which means a sparse distgram
    {
        if(std::isinf(i))
        continue;
        auto dist_i=i;
        int index=ceil(dist_i*5);
        if(distance.find(index)==distance.cend())
        {
            vector<float> vec;
            vec.push_back(dist_i);
            distance.insert(pair<int,vector<float>>({index,vec}));
            key_vec.push_back(index);
        }
        else
        {
            auto &vec=distance.find(index)->second;
            vec.push_back(dist_i);
        }
        count++;
    }
    //sort(key_vec.begin(),key_vec.end());
    sort_and_deduplication(key_vec);
    vector<float> PDF,mean_dist;
   // cout<<"key vec:";
    // for(auto i:key_vec)
    // cout<<i<<' ';
    // cout<<endl;
    int key_index=0;
    for(auto i=distance.begin();i!=distance.end();i++,key_index++)
    {   
        auto vec_i=i->second;
        int key=i->first;
        int key_future=9999;
        if(key_index+1<key_vec.size())
            key_future=key_vec.at(key_index+1);
        //cout<<vec_i.size()<<count<<endl;
        float p=0;
        float mean_=0;
        float group_size=0;
        do// while(key_future-key==1&&i!=distance.cend())
        {
            vec_i=i->second;
            p+=float(vec_i.size())/count;
            group_size+=vec_i.size();
            for(auto j:vec_i)
            mean_+=j;
            key=i->first;
            if(key_index+1<key_vec.size())
            key_future=key_vec.at(key_index+1);
            else key_future=9999;
            //cout<<key<<' '<<key_future<<' '<<key_index<<endl;
            if(key_future-key==1)
            {
                i++;
               key_index++;
            }
            
        }while(key_future-key==1&&i!=distance.cend());
        //cout<<p<<endl;
        PDF.push_back(p);
        // for(auto j:vec_i)
        // mean_+=j;
        mean_dist.push_back(mean_/group_size);
    }
    float max_pdf=0,CDF=0;
    for(int i=0;i!=PDF.size();i++)
    {  
        CDF+=PDF.at(i);
        //std::cout<<i<<':'<<"dist:"<<mean_dist.at(i)<<" PDF:"<<PDF.at(i)<<" CDF"<<CDF<<"PDF SIZE:"<<PDF.size()<<std::endl;
        if(CDF>0.3)
        break;
        //std::cout<<i<<':'<<"dist:"<<mean_dist.at(i)<<" PDF:"<<PDF.at(i)<<" CDF"<<CDF<<std::endl;
        if((PDF.at(i)<=0.15)&&(PDF.at(i)>max_pdf))
        {
            max_pdf=PDF.at(i);
            pole_distance=mean_dist.at(i);
        }

    }
    return pole_distance;
}