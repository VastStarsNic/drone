#include "Filt_Frame.hpp"

using std::cout;

rp_frame::rp_frame(vector<float> ranges, float anglemin, float angleinc)
{
    int i = 0;
    cout<<"init rp_frame instance"<<endl;
    angle.min = anglemin;
    angle.inc = angleinc;
    for(i=0; i<ranges.size(); i++){
        distances.push_back(ranges[i]);
    }
    jump_thres = 0.2;
    pole_width = 3.5;
}

rp_frame::~rp_frame(void)
{
}

void rp_frame::get_jmp_pts(void)
{
    int i = 0;
    dt_point pt;
    float change;
    jmp_pts.clear();
    cout<<"get_jmp_pts"<<endl;
    for(i = 0; i < distances.size()-1 ; i++)
    {
        change = distances[i+1] - distances[i];
        if(change > jump_thres){
            pt.type = 1;
            pt.angle = angle.min + (i+1) * angle.inc;
            pt.distance = distances[i+1];
            jmp_pts.push_back(pt);
        }
        else if(change < -jump_thres){
            pt.type = -1;
            pt.angle = angle.min + i * angle.inc;
            pt.distance = distances[i];
            jmp_pts.push_back(pt);
        }
    }
    cout<<jmp_pts.size()<<endl;
}

float rp_frame::get_pole(void)
{
    vector<dt_point> pole_pair;
    dt_point pole_center;
    float dist;
    int index;
    cout<<"get_pole"<<":";
    get_jmp_pts();
    remove_outline();
    pole_pair = sel_best_pair();
    if(pole_pair.empty()){
        cout<<"empty"<<endl;
        return 0.0;
    }
    pole_center.angle = (pole_pair[0].angle + pole_pair[1].angle) / 2;
    index = int((pole_center.angle - angle.min) / angle.inc + 0.2);
    dist = distances[index];
    pole_center.type = 0;
    pole_center.distance = dist;
    cout<<dist<<endl;
    return dist;
}

void rp_frame::remove_outline(void)
{
    int i = 0;
    float gap_r, gap_a, mid_gap;
    vector<float> gaps;
    vector<float> sortgaps;
    vector<dt_point> alter;
    float mid_dist, mid_angle;
    float dist_var, angle_var, width_var;
    options.clear();
    cout<<"remove_outline"<<endl;
    cout<<"part1_possible_poles: "<<endl;
    for(i = 0; i < jmp_pts.size()-1; i++)
    {
        if(jmp_pts[i].type == -1 && jmp_pts[i+1].type == 1){
            // dist_var = jmp_pts[i].distance - jmp_pts[i+1].distance;
            // if(dist_var > - 0.02 && dist_var < 0.02){
                angle_var = jmp_pts[i+1].angle - jmp_pts[i].angle + angle.inc;
                width_var = angle_var / 1.8 * pi * jmp_pts[i].distance - pole_width;
                if( width_var > -pole_width && width_var < pole_width ){
                    options.push_back(jmp_pts[i]);
                    options.push_back(jmp_pts[i+1]);
                }
            // }
        }
    }
    cout<<options.size()<<endl;
    cout<<"part2_detect_outline: "<<endl;
    gaps.clear();
    for(i = 2; i < options.size(); i += 2)
    {
        gap_r = options[i].distance - options[i-1].distance;
        gap_a = gap_r < 0 ? options[i].distance : options[i-1].distance;
        gap_a *= options[i].angle - options[i-1].angle;
        gaps.push_back(gap_r*gap_r + gap_a*gap_a);
        cout<<gap_r*gap_r + gap_a*gap_a<<",";
    }
    cout<<endl;
    sortgaps = gaps;
    sort(sortgaps.begin(), sortgaps.end());
    mid_gap = sortgaps[int(sortgaps.size()/2)];
    thres_gap = (sortgaps[gaps.size() - 1] - sortgaps[0]) / 4 + mid_gap;
    cout<<"mid:"<<mid_gap<<"tsh:"<<thres_gap<<";";
    if(gaps[0] > thres_gap){
        alter.push_back(options[0]);
        alter.push_back(options[1]);
    }
    for(i = 1; i < gaps.size(); i++)
    {
        if(gaps[i-1] > thres_gap && gaps[i] > thres_gap){
            alter.push_back(options[i*2]);
            alter.push_back(options[i*2+1]);
        }
    }
    options = alter;
    cout<<alter.size()<<options.size()<<endl;
}

vector<dt_point> rp_frame::sel_best_pair(void)
{
    int i = 0;
    vector<dt_point> pole;
    cout<<"sel_best_pair:"<<"from";
    if(options.empty()){
        cout<<"empty"<<endl;
        pole.clear();
        return pole;
    }
    cout<<options.size()<<endl;
    pole.push_back(options[0]);
    pole.push_back(options[1]);
    for(i = 2; i < options.size(); i += 2)
    {
        if(pole[0].distance > options[i].distance){
            pole[0] = options[i];
            pole[1] = options[i+1];
        }
    }
    cout<<"select:"<<pole[0].distance<<","<<pole[1].distance<<endl;
    return pole;
}
