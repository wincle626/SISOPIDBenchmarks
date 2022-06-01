// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University
#include <mex.hpp>
#include "mexAdapter.hpp"
#include "c_RightHalfPlaneZero_SISO.h"
#include "./fpx/floatx.hpp"
#define EXPONENT (unsigned int)6
#define SIGNIFICAND (unsigned int)14
typedef flx::floatx<EXPONENT, SIGNIFICAND> FPX_T;
class MexFunction : public matlab::mex::Function {
    matlab::data::ArrayFactory factory;
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        FPX_T setpoint = (FPX_T) inputs[0][0];
        FPX_T dt = (FPX_T) inputs[1][0];
        FPX_T X1 = (FPX_T) inputs[2][0];
        FPX_T X2 = (FPX_T) inputs[3][0];
        FPX_T X3 = (FPX_T) inputs[4][0];
        FPX_T X4 = (FPX_T) inputs[5][0];
        FPX_T X5 = (FPX_T) inputs[6][0];
        FPX_T X6 = (FPX_T) inputs[7][0];
        FPX_T X7 = (FPX_T) inputs[8][0];
        FPX_T X8 = (FPX_T) inputs[9][0];
        FPX_T X = (FPX_T) inputs[10][0];
        FPX_T E = (FPX_T) inputs[11][0];
        FPX_T P = (FPX_T) inputs[12][0];
        FPX_T I = (FPX_T) inputs[13][0];
        FPX_T D = (FPX_T) inputs[14][0];
        FPX_T PID = (FPX_T) inputs[15][0];
        FPX_T Kp = (FPX_T) inputs[16][0];
        FPX_T Ki = (FPX_T) inputs[17][0];
        FPX_T Kd = (FPX_T) inputs[18][0];
        RightHalfPlaneZero_SISO<FPX_T>(setpoint, dt, X1, X2, X3, X4,
             X5, X6, X7, X8, X, E, P, I, D, PID, Kp, Ki, Kd);
        outputs[0] = factory.createScalar((double)E);
        outputs[1] = factory.createScalar((double)P);
        outputs[2] = factory.createScalar((double)I);
        outputs[3] = factory.createScalar((double)D);
        outputs[4] = factory.createScalar((double)PID);
        outputs[5] = factory.createScalar((double)X1);
        outputs[6] = factory.createScalar((double)X2);
        outputs[7] = factory.createScalar((double)X3);
        outputs[8] = factory.createScalar((double)X4);
        outputs[9] = factory.createScalar((double)X5);
        outputs[10] = factory.createScalar((double)X6);
        outputs[11] = factory.createScalar((double)X7);
        outputs[12] = factory.createScalar((double)X8);
        outputs[13] = factory.createScalar((double)X);
    }
    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;
        if (inputs.size() != 19) {
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
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 1 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[1].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 2 must be a scalar") }));
        }
        
        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 2 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[2].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 3 must be a scalar") }));
        }
        
        if (inputs[2].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[2].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 3 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[3].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 4 must be a scalar") }));
        }
        
        if (inputs[3].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[3].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 4 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[4].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 5 must be a scalar") }));
        }
        
        if (inputs[4].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[4].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 5 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[5].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 6 must be a scalar") }));
        }
        
        if (inputs[5].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[5].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 6 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[6].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 7 must be a scalar") }));
        }
        
        if (inputs[6].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[6].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 7 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[7].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 8 must be a scalar") }));
        }
        
        if (inputs[7].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[7].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 8 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[8].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 9 must be a scalar") }));
        }
        
        if (inputs[8].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[8].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 9 must be a noncomplex scalar FPX_T") }));
        }
        if (inputs[9].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 10 must be a scalar") }));
        }
        
        if (inputs[9].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[9].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input data 10 must be a noncomplex scalar FPX_T") }));
        }
    }
};

