#pragma once
#define MAX_ACTIVE_FRAMES 100

#include "util/globalCalib.h"
#include "vector"

#include <iostream>
#include <fstream>
#include "util/NumType.h"
#include "FullSystem/Residuals.h"
#include "util/ImageAndExposure.h"

struct FrameHessian
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EFFrame *efFrame;

    // constant info & pre-calculated values
    //DepthImageWrap* frame;
    FrameShell *shell;

    Eigen::Vector3f *dI;               // trace, fine tracking. Used for direction select (not for gradient histograms etc.)
    Eigen::Vector3f *dIp[PYR_LEVELS];  // coarse tracking / coarse initializer. NAN in [0] only.
    float *absSquaredGrad[PYR_LEVELS]; // only used for pixel select (histograms etc.). no NAN.

    int frameID; // incremental ID for keyframes only!
    static int instanceCounter;
    int idx;

    // Photometric Calibration Stuff
    float frameEnergyTH; // set dynamically depending on tracking residual
    float ab_exposure;

    bool flaggedForMarginalization;

    std::vector<PointHessian *> pointHessians;             // contains all ACTIVE points.
    std::vector<PointHessian *> pointHessiansMarginalized; // contains all MARGINALIZED points (= fully marginalized, usually because point went OOB.)
    std::vector<PointHessian *> pointHessiansOut;          // contains all OUTLIER points (= discarded.).
    std::vector<PointHessian *> potentialPointHessians;
    std::vector<ImmaturePoint *> immaturePoints;

    Mat66 nullspaces_pose;
    Mat42 nullspaces_affine;
    Vec6 nullspaces_scale;

    // variable info.
    SE3 worldToCam_evalPT;
    Vec10 state_zero;
    Vec10 state_scaled;
    Vec10 state; // [0-5: worldToCam-leftEps. 6-7: a,b]
    Vec10 step;
    Vec10 step_backup;
    Vec10 state_backup;

    EIGEN_STRONG_INLINE const SE3 &get_worldToCam_evalPT() const { return worldToCam_evalPT; }
    EIGEN_STRONG_INLINE const Vec10 &get_state_zero() const { return state_zero; }
    EIGEN_STRONG_INLINE const Vec10 &get_state() const { return state; }
    EIGEN_STRONG_INLINE const Vec10 &get_state_scaled() const { return state_scaled; }
    EIGEN_STRONG_INLINE const Vec10 get_state_minus_stateZero() const { return get_state() - get_state_zero(); }

    // precalc values
    SE3 PRE_worldToCam;
    SE3 PRE_camToWorld;
    std::vector<FrameFramePrecalc, Eigen::aligned_allocator<FrameFramePrecalc>> targetPrecalc;
    MinimalImageB3 *debugImage;

    inline Vec6 w2c_leftEps() const { return get_state_scaled().head<6>(); }
    inline AffLight aff_g2l() const { return AffLight(get_state_scaled()[6], get_state_scaled()[7]); }
    inline AffLight aff_g2l_0() const { return AffLight(get_state_zero()[6] * SCALE_A, get_state_zero()[7] * SCALE_B); }

    void setStateZero(Vec10 state_zero);
    inline void setState(Vec10 state)
    {

        this->state = state;
        state_scaled.segment<3>(0) = SCALE_XI_TRANS * state.segment<3>(0);
        state_scaled.segment<3>(3) = SCALE_XI_ROT * state.segment<3>(3);
        state_scaled[6] = SCALE_A * state[6];
        state_scaled[7] = SCALE_B * state[7];
        state_scaled[8] = SCALE_A * state[8];
        state_scaled[9] = SCALE_B * state[9];

        PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
        PRE_camToWorld = PRE_worldToCam.inverse();
        //setCurrentNullspace();
    };
    inline void setStateScaled(Vec10 state_scaled)
    {

        this->state_scaled = state_scaled;
        state.segment<3>(0) = SCALE_XI_TRANS_INVERSE * state_scaled.segment<3>(0);
        state.segment<3>(3) = SCALE_XI_ROT_INVERSE * state_scaled.segment<3>(3);
        state[6] = SCALE_A_INVERSE * state_scaled[6];
        state[7] = SCALE_B_INVERSE * state_scaled[7];
        state[8] = SCALE_A_INVERSE * state_scaled[8];
        state[9] = SCALE_B_INVERSE * state_scaled[9];

        PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
        PRE_camToWorld = PRE_worldToCam.inverse();
        //setCurrentNullspace();
    };
    inline void setEvalPT(SE3 worldToCam_evalPT, Vec10 state)
    {

        this->worldToCam_evalPT = worldToCam_evalPT;
        setState(state);
        setStateZero(state);
    };

    inline void setEvalPT_scaled(SE3 worldToCam_evalPT, AffLight aff_g2l)
    {
        Vec10 initial_state = Vec10::Zero();
        initial_state[6] = aff_g2l.a;
        initial_state[7] = aff_g2l.b;
        this->worldToCam_evalPT = worldToCam_evalPT;
        setStateScaled(initial_state);
        setStateZero(this->get_state());
    };

    void release();

    inline ~FrameHessian()
    {
        assert(efFrame == 0);
        release();
        instanceCounter--;
        for (int i = 0; i < pyrLevelsUsed; i++)
        {
            delete[] dIp[i];
            delete[] absSquaredGrad[i];
        }

        if (debugImage != 0)
            delete debugImage;
    };
    inline FrameHessian()
    {
        instanceCounter++;
        flaggedForMarginalization = false;
        frameID = -1;
        efFrame = 0;
        frameEnergyTH = 8 * 8 * patternNum;

        debugImage = 0;
    };

    void makeImages(float *color, CalibHessian *HCalib);

    inline Vec10 getPrior()
    {
        Vec10 p = Vec10::Zero();
        if (frameID == 0)
        {
            p.head<3>() = Vec3::Constant(setting_initialTransPrior);
            p.segment<3>(3) = Vec3::Constant(setting_initialRotPrior);
            if (setting_solverMode & SOLVER_REMOVE_POSEPRIOR)
                p.head<6>().setZero();

            p[6] = setting_initialAffAPrior;
            p[7] = setting_initialAffBPrior;
        }
        else
        {
            if (setting_affineOptModeA < 0)
                p[6] = setting_initialAffAPrior;
            else
                p[6] = setting_affineOptModeA;

            if (setting_affineOptModeB < 0)
                p[7] = setting_initialAffBPrior;
            else
                p[7] = setting_affineOptModeB;
        }
        p[8] = setting_initialAffAPrior;
        p[9] = setting_initialAffBPrior;
        return p;
    }

    inline Vec10 getPriorZero()
    {
        return Vec10::Zero();
    }
};