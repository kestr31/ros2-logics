# Public libs
import numpy as np

# Private libs
from .BaseModules.VirtualTarget  import distToPath

#.. Calc. Cost
def Calc_PF_cost(W1, W2, nextWPidx, WPs, Pos, Vn, Throttle):
    prevWPidx    =   max(nextWPidx - 1, 0)
    d2WP        =   10000.
    for i_wp in range(prevWPidx + 1):
        nearWPidx   =   prevWPidx - i_wp
        nearWP      =   WPs[nearWPidx]
        d2WP_tmp    =   np.linalg.norm(nearWP - Pos)
        if d2WP_tmp > d2WP:
            break
        d2WP        =   d2WP_tmp


    maxWPidx    =   WPs.shape[0] - 1
    arrWPidx    =   np.array([nearWPidx - 1, nearWPidx, nearWPidx + 1, nearWPidx + 2])
    arrWPidx    =   np.where(arrWPidx < 0, 0, arrWPidx)
    arrWPidx    =   np.where(arrWPidx > maxWPidx, maxWPidx, arrWPidx)

    closestPosOnPath    =   np.zeros(3)
    dist_Path   =   10000.
    for i_wpidx in range(arrWPidx.shape[0] - 1):
        nearWPidx   =   arrWPidx[i_wpidx]
        nextWPidx   =   arrWPidx[i_wpidx + 1]
        prevWP      =   WPs[nearWPidx].copy()
        if nearWPidx == nextWPidx:
            relPos  =   prevWP - Pos
            relPos[2]   =   0.
            D_P     =   np.linalg.norm(relPos)
            CPOP    =   prevWP.copy()
        else:
            nextWP  =   WPs[nextWPidx].copy()
            Pos     =   Pos.copy()
            Pos[2]  =   0.
            prevWP[2]   =   0.
            nextWP[2]   =   0.
            D_P, CPOP =   distToPath(Pos, prevWP, nextWP)
        
        if D_P <= dist_Path:
            dist_Path   =   D_P
            closestPosOnPath    =   CPOP
            
    # SpdOnPath  =   max(np.linalg.norm(closestPosOnPath - quad.datGCU.PrevClosetPosOnPath), 1e-08) / quad.datSim.dt
    # quad.datGCU.PrevClosetPosOnPath =   closestPosOnPath.copy()
    # c_Spd           =   1/max(SpdOnPath, 0.5)

    c_d2p           =   dist_Path * dist_Path

    c_ctrl_e        =   Throttle * Throttle
    cost            =   W1 * c_d2p + W2 * c_ctrl_e
    
    return cost, dist_Path