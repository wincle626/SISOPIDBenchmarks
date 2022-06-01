// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University
#include <mex.hpp>
#include "mexAdapter.hpp"
#include "a_MultipleEqualPoles_SISO.h"
#include "./fxp/ap_fixed.h"
#include "./fxp/ap_int.h"
#define BIT_WIDTH (unsigned int)32
#define INTE_WIDTH (unsigned int)16
typedef ap_fixed<BIT_WIDTH,INTE_WIDTH> FXP_T;
class MexFunction : public matlab::mex::Function {
    matlab::data::ArrayFactory factory;
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        double tmp = inputs[0][0];
        FXP_T setpoint = tmp;
        tmp = inputs[1][0];
        FXP_T dt = (FXP_T) tmp;
        tmp = inputs[2][0];
        FXP_T X = (FXP_T) tmp;
        tmp = inputs[3][0];
        FXP_T E = (FXP_T) tmp;
        tmp = inputs[4][0];
        FXP_T P = (FXP_T) tmp;
        tmp = inputs[5][0];
        FXP_T I = (FXP_T) tmp;
        tmp = inputs[6][0];
        FXP_T D = (FXP_T) tmp;
        tmp = inputs[7][0];
        FXP_T PID = (FXP_T) tmp;
        tmp = inputs[8][0];
        FXP_T Kp = (FXP_T) tmp;
        tmp = inputs[9][0];
        FXP_T Ki = (FXP_T) tmp;
        tmp = inputs[10][0];
        FXP_T Kd = (FXP_T) tmp;
        MultipleEqualPoles_SISO<FXP_T>(setpoint, dt, X,
             E, P, I, D, PID, 
             Kp, Ki, Kd);
        outputs[0] = factory.createScalar((double)E);
        outputs[1] = factory.createScalar((double)P);
        outputs[2] = factory.createScalar((double)I);
        outputs[3] = factory.createScalar((double)D);
        outputs[4] = factory.createScalar((double)PID);
        outputs[5] = factory.createScalar((double)X);
    }
    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;
        if (inputs.size() != 11) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Ten inputs required") }));
        }
        if (inputs[0].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 1 must be a scalar") }));
        }
        
        if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 1 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[1].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 2 must be a scalar") }));
        }
        
        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 2 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[2].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 3 must be a scalar") }));
        }
        
        if (inputs[2].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[2].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 3 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[3].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 4 must be a scalar") }));
        }
        
        if (inputs[3].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[3].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 4 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[4].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 5 must be a scalar") }));
        }
        
        if (inputs[4].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[4].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 5 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[5].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 6 must be a scalar") }));
        }
        
        if (inputs[5].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[5].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 6 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[6].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 7 must be a scalar") }));
        }
        
        if (inputs[6].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[6].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 7 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[7].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 8 must be a scalar") }));
        }
        
        if (inputs[7].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[7].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 8 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[8].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 9 must be a scalar") }));
        }
        
        if (inputs[8].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[8].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 9 must be a noncomplex scalar FXP_T") }));
        }
        if (inputs[9].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 10 must be a scalar") }));
        }
        
        if (inputs[9].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[9].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 10 must be a noncomplex scalar FXP_T") }));
        }
    }
};

