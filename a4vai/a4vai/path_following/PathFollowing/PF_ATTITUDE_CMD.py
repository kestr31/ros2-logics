# Public libs
import numpy as np
from math import pi
R2D = 180. / pi

# Private libs
from .BaseModules.ParamsOffBoardCtrl import DataGCU
from .BaseModules.CommonFunctions import Get_Vec2AzimElev, Get_Euler2DCM
from .BaseModules.VirtualTarget import Calc_VirTgPos
from .BaseModules.Kinematics import Kinematics
from .BaseModules.GCU_Main import Guid_pursuit, SpdCtrller, AccCmdToCtrlCmd

# temp
from .PF_Cost import Calc_PF_cost

class PF_ATTITUDE_CMD_MOD():
    def __init__(self, dt) -> None:
    #.. Parameters
        self.GCUParams  =   DataGCU(dt)

    #.. NDO variables
        self.FbCmd      =   np.array([0., 0., -self.GCUParams.Mass * self.GCUParams.g0])
        self.z_NDO      =   np.zeros(3)
        self.z_NDO_in = [0.0, 0.0, 0.0]
        self.lx_NDO     =   4.
        self.ly_NDO     =   4.
        self.lz_NDO     =   0.2
        self.outNDO     =   np.zeros(3)
        self.a_drag_n   =   np.zeros(3)
        self.firstTime = True

    #.. temp
        self.Flag_Write =   False # True
        self.total_cost =   0.
        #self.datalogFile    =   open("/root/ros_ws/src/a4vai/a4vai/path_following/sharedir/datalog.txt",'w')
        pass

    def PF_ATTITUDE_CMD_Module(self, timestemp, PlannedX, PlannedY, PlannedZ, PlannnedIndex, Pos, Vn, AngEuler, Acc_disturb, z_NDO_past, LAD=2., SPDCMD=2.):
        print("###########11111!!!!###########")

        # self.GCUParams.dt_GCU = 1.
        # Vn = np.zeros(3)
        # self.FbCmd = np.zeros(3)
        # AngEuler = np.zeros(3)
        # self.GCUParams.Mass = 1.
        # self.GCUParams.rho = 1.
        # self.GCUParams.Sref = 1.
        # self.GCUParams.CD = 1.
        # self.GCUParams.g0 = 1.
        # z_NDO_past = np.zeros(3)


        outNDO, z_NDO_out = self.NDO_main(self.GCUParams.dt_GCU, Vn, self.FbCmd, AngEuler, self.GCUParams.Mass, \
            self.GCUParams.rho, self.GCUParams.Sref, self.GCUParams.CD, self.GCUParams.g0, z_NDO_past)
        print("###########2222222###########")
        if Acc_disturb[0] == 0.:
            Acc_disturb =   self.outNDO + self.a_drag_n
        TargetThrust, TargetAttitude, TargetPosition, TargetYaw = \
            self.PF_main(timestemp, PlannedX, PlannedY, PlannedZ, PlannnedIndex, Pos, Vn, AngEuler, Acc_disturb, LAD, SPDCMD)
        print("##########333333###########")

        return TargetThrust, TargetAttitude.tolist(), TargetPosition.tolist(), TargetYaw, outNDO.tolist(), z_NDO_out.tolist()

    def PF_main(self, timestemp, PlannedX, PlannedY, PlannedZ, PlannnedIndex, Pos, Vn, AngEuler, Acc_disturb, LAD=2., SPDCMD=2.):
        #.. Guid. Params.
            self.GCUParams.lookAheadDist    =      LAD
            print("##########444444###########")
            self.GCUParams.desSpd       =   SPDCMD
            # self.GCUParamsreachDist     =   self.GCUParams.lookAheadDist
            self.GCUParamsreachDist     =   3.
        
        #.. inputs
            WPs         =   np.array([PlannedX, PlannedY, PlannedZ]).transpose()
            nextWPidx   =   PlannnedIndex
            Pos         =   np.array(Pos)
            Vn          =   np.array(Vn)
            AngEuler    =   np.array(AngEuler)
            Acc_disturb =   np.array(Acc_disturb)
            print("##########55555###########")

        #.. Virtual Target
            # function & output
            tgPos       =   Calc_VirTgPos(Pos, nextWPidx, WPs, LAD)
            print("##########666###########")

        #.. Kinematics
            # input
            tgVn        =   np.array([0., 0., 0.])
            # function & output
            LOSazim, LOSelev, dLOSvec, reldist, tgo  =   Kinematics(tgPos, tgVn, Pos, Vn)
            print("##########666###########")

        #.. Guidance
            Kgain           =   self.GCUParams.Kgain_guidPursuit
            AccLim          =   self.GCUParams.AccLim
            # function & output
            AccCmdw_Lat     =   Guid_pursuit(Kgain, tgo, LOSazim, LOSelev, Vn, AccLim)
            AccCmdw         =   AccCmdw_Lat
            print("##########666###########")

        #.. speed control module
            # vars. for inputs
            magAccCmdLat    =   np.linalg.norm(AccCmdw_Lat)
            desSpd          =   max(self.GCUParams.desSpd - self.GCUParams.desSpd_weight*magAccCmdLat, self.GCUParams.desSpd * 0.5)

            # input
            Kp              =   self.GCUParams.Kp_vel
            Ki              =   self.GCUParams.Ki_vel
            Kd              =   self.GCUParams.Kd_vel
            int_err_spd     =   self.GCUParams.int_err_spd
            prev_err        =   self.GCUParams.prev_val
            dt              =   self.GCUParams.dt_GCU
            # function & output
            AccCmdw_Ax, int_err_spd, prev_err   =   SpdCtrller(Kp, Ki, Kd, int_err_spd, prev_err, dt, Vn, desSpd)
            self.GCUParams.int_err_spd     =   int_err_spd
            self.GCUParams.prev_val        =   prev_err
            AccCmdw[0]      =   AccCmdw_Ax[0]
        
        #.. Disturbance Rejection
            # vars. for inputs
            psi, gam        =   Get_Vec2AzimElev(Vn)
            angI2W          =   np.array([0., -gam, psi])
            cI_W            =   Get_Euler2DCM(angI2W)
            cW_I            =   np.transpose(cI_W)

            AccCmdn         =   np.dot(cW_I, AccCmdw)
            AccCmdn_total   =   AccCmdn - Acc_disturb

            # min, max
            AccCmdn_total     =   np.where(AccCmdn_total > AccLim, AccLim, AccCmdn_total)
            AccCmdn_total     =   np.where(AccCmdn_total < -AccLim, -AccLim, AccCmdn_total)
            
        #.. Accel. to Att.
            # input
            throttle_Hover  =   self.GCUParams.throttle_Hover
            mass            =   self.GCUParams.Mass
            g               =   self.GCUParams.g0
            # function & output
            AttCmd, FbCmd   =   AccCmdToCtrlCmd(AccCmdn_total, LOSazim, throttle_Hover, mass, g)
            if (AngEuler[2] - AttCmd[2])*R2D < -200.:
                AttCmd[2]   =   AttCmd[2] - 2*pi
            if (AngEuler[2] - AttCmd[2])*R2D > 200.:
                AttCmd[2]   =   AttCmd[2] + 2*pi
            # function & output
            totalFbCmd      =   np.linalg.norm(FbCmd)
            ThrustHover     =   throttle_Hover / (mass * g)
            ThrustCmd       =   totalFbCmd * ThrustHover

        #.. temp
            # calc. cost
            W1      =   self.GCUParams.W1_cost
            W2      =   self.GCUParams.W2_cost
            cost, dist_Path =   Calc_PF_cost(W1, W2, nextWPidx, WPs, Pos, Vn, ThrustCmd)
            c_d2p   =    dist_Path * dist_Path
            self.total_cost     =   self.total_cost + cost

            if self.Flag_Write == 1:    
                if nextWPidx < WPs.shape[0]:
                #.. Save Data
                    AccCmdw_total   =   np.dot(cI_W, AccCmdn_total)
                    outNDOw         =   np.dot(cI_W, Acc_disturb)
                    Spd     =   np.linalg.norm(Vn)
                    Data = "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" %(
                        timestemp, AttCmd[0]*R2D, AttCmd[1]*R2D, AttCmd[2]*R2D, AngEuler[0]*R2D, AngEuler[1]*R2D, AngEuler[2]*R2D,
                        AccCmdw_total[0], AccCmdw_total[1], AccCmdw_total[2], self.outNDO[0], self.outNDO[1], self.outNDO[2],
                        Pos[0], Pos[1], Pos[2], tgPos[0], tgPos[1], tgPos[2], 
                        cost, self.total_cost, dist_Path, outNDOw[0], outNDOw[1], outNDOw[2],
                        Spd, desSpd, self.GCUParams.lookAheadDist, self.GCUParams.desSpd, self.a_drag_n[1], self.a_drag_n[2],
                        )
                    self.datalogFile.write(Data)
                else:
                    self.datalogFile.close()


            return ThrustCmd, AttCmd, tgPos, LOSazim
        

    def NDO_main(self, dt, Vn, FbCmd, AngEuler, mass, rho, Sref, CD, g, z_NDO_past):
        print("-------- 111111111111111111---------------------")
        # Calc. Aero. Force
        psi, gam        =   Get_Vec2AzimElev(Vn)
        angI2W          =   np.array([0., -gam, psi])
        cI_W            =   Get_Euler2DCM(angI2W)
        cW_I            =   np.transpose(cI_W)

        Spd             =   np.linalg.norm(Vn)
        qbar            =   0.5 * rho * Spd * Spd
        a_drag_w        =   np.array([ -qbar*Sref*CD / mass, 0., 0. ])
        self.a_drag_n   =   np.dot(cW_I, a_drag_w)
        
        # Calc. Acc. Cmd.
        cI_B            =   Get_Euler2DCM(AngEuler)
        cB_I            =   np.transpose(cI_B)
        Acmdn           =   np.dot(cB_I, FbCmd / mass) + np.array([0., 0., g]) + self.a_drag_n 
        print("-------- 2222222222222222---------------------")
        dz_NDO      =   np.zeros(3)
        if self.firstTime is True :
            pass
        else : 
            self.z_NDO = z_NDO_past
        dz_NDO[0]   =   -self.lx_NDO*self.z_NDO[0] - self.lx_NDO * \
            (self.lx_NDO*Vn[0] + Acmdn[0])
        dz_NDO[1]   =   -self.ly_NDO*self.z_NDO[1] - self.ly_NDO * \
            (self.ly_NDO*Vn[1] + Acmdn[1])
        dz_NDO[2]   =   -self.lz_NDO*self.z_NDO[2] - self.lz_NDO * \
            (self.lz_NDO*Vn[2] + Acmdn[2])
        
        self.outNDO[0]    =   self.z_NDO[0] + self.lx_NDO*Vn[0]
        self.outNDO[1]    =   self.z_NDO[1] + self.ly_NDO*Vn[1]
        self.outNDO[2]    =   self.z_NDO[2] + self.lz_NDO*Vn[2]

        self.z_NDO  =   self.z_NDO + dz_NDO*dt

        return self.outNDO, self.z_NDO