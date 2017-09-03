//
// Created by jbs on 17. 9. 3.
//

#ifndef PROG_AGENT_PROB_WORKSPACE_H
#define PROG_AGENT_PROB_WORKSPACE_H



class workspace{

public:

    // total Nrow x Ncol cells with cell dim=ds
    double ds;
    int Nx, Ny;
    idx target_loc;
    pos_map m;
    workspace(int ,int ,const idx& ,double);

};

workspace::workspace(int nx, int ny, const idx &target_idx, double ds) {
    Ny = ny;
    Nx = nx;
    this->ds = ds;
// location will be given
    get<0>(target_loc) = get<0>(target_idx);
    get<1>(target_loc) = get<1>(target_idx);

    for (int i = 1; i <= Nx; i++)
        for (int j = 1; j <= Ny; j++) {
//pos cur_pos(ds/2+ds*(i-1),ds/2+ds*(j-1));
            pos cur_pos;
            cur_pos.x = ds / 2 + ds * (i - 1);
            cur_pos.y = ds / 2 + ds * (j - 1);
            idx cur_idx(i, j);
            m.insert(pair<idx, pos>(cur_idx, cur_pos));
        }
}


#endif //PROG_AGENT_PROB_WORKSPACE_H
