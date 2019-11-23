//
// Created by wang on 19-11-23.
//

#ifndef SRC_CSMMATCHER_H
#define SRC_CSMMATCHER_H

#include <csm/csm_all.h>
#include <Eigen/Core>
#include <iostream>
class CsmMatcher {
public:
    CsmMatcher(){
        m_prevLDP = NULL;
        SetPIICPParams();
    }
    //设置PI-ICP的参数
    void SetPIICPParams();
    Eigen::Vector3d  PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                           Eigen::Vector3d tmprPose);
    //进行PI-ICP需要的变量
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;
};


#endif //SRC_CSMMATCHER_H
