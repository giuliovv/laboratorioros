//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynsim_2link_planar.cpp
//
// Code generated for Simulink model 'dynsim_2link_planar'.
//
// Model version                  : 1.130
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 17 16:48:26 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "dynsim_2link_planar.h"
#include "dynsim_2link_planar_private.h"

// Block signals (default storage)
B_dynsim_2link_planar_T dynsim_2link_planar_B;

// Continuous states
X_dynsim_2link_planar_T dynsim_2link_planar_X;

// Block states (default storage)
DW_dynsim_2link_planar_T dynsim_2link_planar_DW;

// Real-time model
RT_MODEL_dynsim_2link_planar_T dynsim_2link_planar_M_ =
  RT_MODEL_dynsim_2link_planar_T();
RT_MODEL_dynsim_2link_planar_T *const dynsim_2link_planar_M =
  &dynsim_2link_planar_M_;

// Forward declaration for local functions
static void dynsim_2link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0);
static void dynsim_2link_pla_emxInit_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions);
static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_2link__T
  *emxArray, int32_T oldNumel);
static void dynsim_2lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray, int32_T numDimensions);
static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  *emxArray, int32_T oldNumel);
static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T ax[3]);
static void dynsim_2link_planar_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_a(const
  c_rigidBodyJoint_dynsim_2link_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T T[16]);
static void dynsim_2link_planar_tforminv(const real_T T[16], real_T Tinv[16]);
static void dynsim_2lin_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void dynsim_2link_pla_emxInit_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions);
static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_2link__T
  *emxArray, int32_T oldNumel);
static void dynsim_2link_pla_emxFree_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray);
static void dynsim_2link_pla_emxFree_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray);
static void dynsim_2lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(n_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], emxArray_real_T_dynsim_2link__T *H,
  emxArray_real_T_dynsim_2link__T *lambda);
static void RigidBodyTreeDynamics_inverseDy(n_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[36], real_T
  tau[6]);
static void matlabCodegenHa_awtnueqcp3y53m2(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2link_T
  *pStruct);
static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void matlabCodegenHandl_awtnueqcp3y5(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2link_T
  *pStruct);
static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static l_robotics_manip_internal_Rig_T *dynsim_2lin_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim_2l_RigidBody_RigidBody_a
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim_2_RigidBody_RigidBody_aw
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_awt
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsim_RigidBody_RigidBody_awtn
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *dynsi_RigidBody_RigidBody_awtnu
  (l_robotics_manip_internal_Rig_T *obj);
static m_robotics_manip_internal_Rig_T *dyns_RigidBody_RigidBody_awtnue
  (m_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (n_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Rig_T *iobj_0,
   l_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Rig_T
   *iobj_2, l_robotics_manip_internal_Rig_T *iobj_3,
   l_robotics_manip_internal_Rig_T *iobj_4, l_robotics_manip_internal_Rig_T
   *iobj_5);

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 12;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  dynsim_2link_planar_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  dynsim_2link_planar_step();
  dynsim_2link_planar_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  dynsim_2link_planar_step();
  dynsim_2link_planar_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void dynsim_2link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_dynsim_2link_planar_16.getLatestMessage
    (&dynsim_2link_planar_B.b_varargout_2);
  memcpy(&varargout_2_Data[0], &dynsim_2link_planar_B.b_varargout_2.Data[0],
         sizeof(real_T) << 7U);
  *varargout_2_Data_SL_Info_Curren =
    dynsim_2link_planar_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    dynsim_2link_planar_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    dynsim_2link_planar_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &dynsim_2link_planar_B.b_varargout_2.Layout.Dim[0], sizeof
         (SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    dynsim_2link_planar_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    dynsim_2link_planar_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void dynsim_2link_pla_emxInit_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_dynsim_2link__T *emxArray;
  *pEmxArray = (emxArray_real_T_dynsim_2link__T *)malloc(sizeof
    (emxArray_real_T_dynsim_2link__T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_2link_planar_B.i_oj = 0; dynsim_2link_planar_B.i_oj <
       numDimensions; dynsim_2link_planar_B.i_oj++) {
    emxArray->size[dynsim_2link_planar_B.i_oj] = 0;
  }
}

static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_2link__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_2link_planar_B.newNumel = 1;
  for (dynsim_2link_planar_B.i_a = 0; dynsim_2link_planar_B.i_a <
       emxArray->numDimensions; dynsim_2link_planar_B.i_a++) {
    dynsim_2link_planar_B.newNumel *= emxArray->size[dynsim_2link_planar_B.i_a];
  }

  if (dynsim_2link_planar_B.newNumel > emxArray->allocatedSize) {
    dynsim_2link_planar_B.i_a = emxArray->allocatedSize;
    if (dynsim_2link_planar_B.i_a < 16) {
      dynsim_2link_planar_B.i_a = 16;
    }

    while (dynsim_2link_planar_B.i_a < dynsim_2link_planar_B.newNumel) {
      if (dynsim_2link_planar_B.i_a > 1073741823) {
        dynsim_2link_planar_B.i_a = MAX_int32_T;
      } else {
        dynsim_2link_planar_B.i_a <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_2link_planar_B.i_a), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = dynsim_2link_planar_B.i_a;
    emxArray->canFreeData = true;
  }
}

static void dynsim_2lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynsim_2_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_dynsim_2_T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynsim_2_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynsim_2link_plan_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_2link_planar_B.i_o2 = 0; dynsim_2link_planar_B.i_o2 <
       numDimensions; dynsim_2link_planar_B.i_o2++) {
    emxArray->size[dynsim_2link_planar_B.i_o2] = 0;
  }
}

static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_2link_planar_B.newNumel_a = 1;
  for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <
       emxArray->numDimensions; dynsim_2link_planar_B.i_i++) {
    dynsim_2link_planar_B.newNumel_a *= emxArray->size[dynsim_2link_planar_B.i_i];
  }

  if (dynsim_2link_planar_B.newNumel_a > emxArray->allocatedSize) {
    dynsim_2link_planar_B.i_i = emxArray->allocatedSize;
    if (dynsim_2link_planar_B.i_i < 16) {
      dynsim_2link_planar_B.i_i = 16;
    }

    while (dynsim_2link_planar_B.i_i < dynsim_2link_planar_B.newNumel_a) {
      if (dynsim_2link_planar_B.i_i > 1073741823) {
        dynsim_2link_planar_B.i_i = MAX_int32_T;
      } else {
        dynsim_2link_planar_B.i_i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_2link_planar_B.i_i), sizeof
                     (f_cell_wrap_dynsim_2link_plan_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynsim_2link_plan_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynsim_2link_plan_T *)newData;
    emxArray->allocatedSize = dynsim_2link_planar_B.i_i;
    emxArray->canFreeData = true;
  }
}

static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr_o = 0; dynsim_2link_planar_B.b_kstr_o < 8;
       dynsim_2link_planar_B.b_kstr_o++) {
    dynsim_2link_planar_B.b_l[dynsim_2link_planar_B.b_kstr_o] =
      tmp[dynsim_2link_planar_B.b_kstr_o];
  }

  dynsim_2link_planar_B.b_bool_c = false;
  if (obj->Type->size[1] == 8) {
    dynsim_2link_planar_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_o - 1 < 8) {
        dynsim_2link_planar_B.kstr_i = dynsim_2link_planar_B.b_kstr_o - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr_i] !=
            dynsim_2link_planar_B.b_l[dynsim_2link_planar_B.kstr_i]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_o++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_2link_planar_B.b_bool_c) {
    guard1 = true;
  } else {
    for (dynsim_2link_planar_B.b_kstr_o = 0; dynsim_2link_planar_B.b_kstr_o < 9;
         dynsim_2link_planar_B.b_kstr_o++) {
      dynsim_2link_planar_B.b_j[dynsim_2link_planar_B.b_kstr_o] =
        tmp_0[dynsim_2link_planar_B.b_kstr_o];
    }

    dynsim_2link_planar_B.b_bool_c = false;
    if (obj->Type->size[1] == 9) {
      dynsim_2link_planar_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_o - 1 < 9) {
          dynsim_2link_planar_B.kstr_i = dynsim_2link_planar_B.b_kstr_o - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr_i] !=
              dynsim_2link_planar_B.b_j[dynsim_2link_planar_B.kstr_i]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_o++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_c) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void dynsim_2link_planar_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void rigidBodyJoint_transformBodyT_a(const
  c_rigidBodyJoint_dynsim_2link_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 5;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.b_f[dynsim_2link_planar_B.b_kstr] =
      tmp[dynsim_2link_planar_B.b_kstr];
  }

  dynsim_2link_planar_B.b_bool_g = false;
  if (obj->Type->size[1] == 5) {
    dynsim_2link_planar_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr - 1 < 5) {
        dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr] !=
            dynsim_2link_planar_B.b_f[dynsim_2link_planar_B.kstr]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_g = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_2link_planar_B.b_bool_g) {
    dynsim_2link_planar_B.b_kstr = 0;
  } else {
    for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 8;
         dynsim_2link_planar_B.b_kstr++) {
      dynsim_2link_planar_B.b_g[dynsim_2link_planar_B.b_kstr] =
        tmp_0[dynsim_2link_planar_B.b_kstr];
    }

    dynsim_2link_planar_B.b_bool_g = false;
    if (obj->Type->size[1] == 8) {
      dynsim_2link_planar_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr - 1 < 8) {
          dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr] !=
              dynsim_2link_planar_B.b_g[dynsim_2link_planar_B.kstr]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_g) {
      dynsim_2link_planar_B.b_kstr = 1;
    } else {
      dynsim_2link_planar_B.b_kstr = -1;
    }
  }

  switch (dynsim_2link_planar_B.b_kstr) {
   case 0:
    memset(&dynsim_2link_planar_B.TJ[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.TJ[0] = 1.0;
    dynsim_2link_planar_B.TJ[5] = 1.0;
    dynsim_2link_planar_B.TJ[10] = 1.0;
    dynsim_2link_planar_B.TJ[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v);
    dynsim_2link_planar_B.result_data[0] = dynsim_2link_planar_B.v[0];
    dynsim_2link_planar_B.result_data[1] = dynsim_2link_planar_B.v[1];
    dynsim_2link_planar_B.result_data[2] = dynsim_2link_planar_B.v[2];
    if (0 <= (*q_size != 0) - 1) {
      dynsim_2link_planar_B.result_data[3] = q_data[0];
    }

    dynsim_2link_planar_B.cth = 1.0 / sqrt((dynsim_2link_planar_B.result_data[0]
      * dynsim_2link_planar_B.result_data[0] +
      dynsim_2link_planar_B.result_data[1] * dynsim_2link_planar_B.result_data[1])
      + dynsim_2link_planar_B.result_data[2] *
      dynsim_2link_planar_B.result_data[2]);
    dynsim_2link_planar_B.v[0] = dynsim_2link_planar_B.result_data[0] *
      dynsim_2link_planar_B.cth;
    dynsim_2link_planar_B.v[1] = dynsim_2link_planar_B.result_data[1] *
      dynsim_2link_planar_B.cth;
    dynsim_2link_planar_B.v[2] = dynsim_2link_planar_B.result_data[2] *
      dynsim_2link_planar_B.cth;
    dynsim_2link_planar_B.cth = cos(dynsim_2link_planar_B.result_data[3]);
    dynsim_2link_planar_B.sth = sin(dynsim_2link_planar_B.result_data[3]);
    dynsim_2link_planar_B.tempR_tmp = dynsim_2link_planar_B.v[1] *
      dynsim_2link_planar_B.v[0] * (1.0 - dynsim_2link_planar_B.cth);
    dynsim_2link_planar_B.tempR_tmp_b = dynsim_2link_planar_B.v[2] *
      dynsim_2link_planar_B.sth;
    dynsim_2link_planar_B.tempR_tmp_n = dynsim_2link_planar_B.v[2] *
      dynsim_2link_planar_B.v[0] * (1.0 - dynsim_2link_planar_B.cth);
    dynsim_2link_planar_B.tempR_tmp_bs = dynsim_2link_planar_B.v[1] *
      dynsim_2link_planar_B.sth;
    dynsim_2link_planar_B.tempR_tmp_l = dynsim_2link_planar_B.v[2] *
      dynsim_2link_planar_B.v[1] * (1.0 - dynsim_2link_planar_B.cth);
    dynsim_2link_planar_B.sth *= dynsim_2link_planar_B.v[0];
    dynsim_2link_planar_cat(dynsim_2link_planar_B.v[0] *
      dynsim_2link_planar_B.v[0] * (1.0 - dynsim_2link_planar_B.cth) +
      dynsim_2link_planar_B.cth, dynsim_2link_planar_B.tempR_tmp -
      dynsim_2link_planar_B.tempR_tmp_b, dynsim_2link_planar_B.tempR_tmp_n +
      dynsim_2link_planar_B.tempR_tmp_bs, dynsim_2link_planar_B.tempR_tmp +
      dynsim_2link_planar_B.tempR_tmp_b, dynsim_2link_planar_B.v[1] *
      dynsim_2link_planar_B.v[1] * (1.0 - dynsim_2link_planar_B.cth) +
      dynsim_2link_planar_B.cth, dynsim_2link_planar_B.tempR_tmp_l -
      dynsim_2link_planar_B.sth, dynsim_2link_planar_B.tempR_tmp_n -
      dynsim_2link_planar_B.tempR_tmp_bs, dynsim_2link_planar_B.tempR_tmp_l +
      dynsim_2link_planar_B.sth, dynsim_2link_planar_B.v[2] *
      dynsim_2link_planar_B.v[2] * (1.0 - dynsim_2link_planar_B.cth) +
      dynsim_2link_planar_B.cth, dynsim_2link_planar_B.tempR);
    for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 3;
         dynsim_2link_planar_B.b_kstr++) {
      dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr + 1;
      dynsim_2link_planar_B.R_p[dynsim_2link_planar_B.kstr - 1] =
        dynsim_2link_planar_B.tempR[(dynsim_2link_planar_B.kstr - 1) * 3];
      dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr + 1;
      dynsim_2link_planar_B.R_p[dynsim_2link_planar_B.kstr + 2] =
        dynsim_2link_planar_B.tempR[(dynsim_2link_planar_B.kstr - 1) * 3 + 1];
      dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr + 1;
      dynsim_2link_planar_B.R_p[dynsim_2link_planar_B.kstr + 5] =
        dynsim_2link_planar_B.tempR[(dynsim_2link_planar_B.kstr - 1) * 3 + 2];
    }

    memset(&dynsim_2link_planar_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 3;
         dynsim_2link_planar_B.b_kstr++) {
      dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr << 2;
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr] =
        dynsim_2link_planar_B.R_p[3 * dynsim_2link_planar_B.b_kstr];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr + 1] =
        dynsim_2link_planar_B.R_p[3 * dynsim_2link_planar_B.b_kstr + 1];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr + 2] =
        dynsim_2link_planar_B.R_p[3 * dynsim_2link_planar_B.b_kstr + 2];
    }

    dynsim_2link_planar_B.TJ[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v);
    memset(&dynsim_2link_planar_B.tempR[0], 0, 9U * sizeof(real_T));
    dynsim_2link_planar_B.tempR[0] = 1.0;
    dynsim_2link_planar_B.tempR[4] = 1.0;
    dynsim_2link_planar_B.tempR[8] = 1.0;
    for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 3;
         dynsim_2link_planar_B.b_kstr++) {
      dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr << 2;
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr] =
        dynsim_2link_planar_B.tempR[3 * dynsim_2link_planar_B.b_kstr];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr + 1] =
        dynsim_2link_planar_B.tempR[3 * dynsim_2link_planar_B.b_kstr + 1];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr + 2] =
        dynsim_2link_planar_B.tempR[3 * dynsim_2link_planar_B.b_kstr + 2];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.b_kstr + 12] =
        dynsim_2link_planar_B.v[dynsim_2link_planar_B.b_kstr] * q_data[0];
    }

    dynsim_2link_planar_B.TJ[3] = 0.0;
    dynsim_2link_planar_B.TJ[7] = 0.0;
    dynsim_2link_planar_B.TJ[11] = 0.0;
    dynsim_2link_planar_B.TJ[15] = 1.0;
    break;
  }

  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 4;
       dynsim_2link_planar_B.b_kstr++) {
    for (dynsim_2link_planar_B.kstr = 0; dynsim_2link_planar_B.kstr < 4;
         dynsim_2link_planar_B.kstr++) {
      dynsim_2link_planar_B.obj_tmp_tmp = dynsim_2link_planar_B.kstr << 2;
      dynsim_2link_planar_B.obj_tmp = dynsim_2link_planar_B.b_kstr +
        dynsim_2link_planar_B.obj_tmp_tmp;
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] = 0.0;
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr];
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr + 4];
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr + 8];
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr + 12];
    }

    for (dynsim_2link_planar_B.kstr = 0; dynsim_2link_planar_B.kstr < 4;
         dynsim_2link_planar_B.kstr++) {
      dynsim_2link_planar_B.obj_tmp_tmp = dynsim_2link_planar_B.kstr << 2;
      dynsim_2link_planar_B.obj_tmp = dynsim_2link_planar_B.b_kstr +
        dynsim_2link_planar_B.obj_tmp_tmp;
      T[dynsim_2link_planar_B.obj_tmp] = 0.0;
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr];
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp + 1] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr + 4];
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp + 2] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr + 8];
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp + 3] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr_h = 0; dynsim_2link_planar_B.b_kstr_h < 5;
       dynsim_2link_planar_B.b_kstr_h++) {
    dynsim_2link_planar_B.b_ju[dynsim_2link_planar_B.b_kstr_h] =
      tmp[dynsim_2link_planar_B.b_kstr_h];
  }

  dynsim_2link_planar_B.b_bool_l = false;
  if (obj->Type->size[1] == 5) {
    dynsim_2link_planar_B.b_kstr_h = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_h - 1 < 5) {
        dynsim_2link_planar_B.kstr_j = dynsim_2link_planar_B.b_kstr_h - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr_j] !=
            dynsim_2link_planar_B.b_ju[dynsim_2link_planar_B.kstr_j]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_h++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_l = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_2link_planar_B.b_bool_l) {
    dynsim_2link_planar_B.b_kstr_h = 0;
  } else {
    for (dynsim_2link_planar_B.b_kstr_h = 0; dynsim_2link_planar_B.b_kstr_h < 8;
         dynsim_2link_planar_B.b_kstr_h++) {
      dynsim_2link_planar_B.b_dh[dynsim_2link_planar_B.b_kstr_h] =
        tmp_0[dynsim_2link_planar_B.b_kstr_h];
    }

    dynsim_2link_planar_B.b_bool_l = false;
    if (obj->Type->size[1] == 8) {
      dynsim_2link_planar_B.b_kstr_h = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_h - 1 < 8) {
          dynsim_2link_planar_B.kstr_j = dynsim_2link_planar_B.b_kstr_h - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr_j] !=
              dynsim_2link_planar_B.b_dh[dynsim_2link_planar_B.kstr_j]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_h++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_l = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_l) {
      dynsim_2link_planar_B.b_kstr_h = 1;
    } else {
      dynsim_2link_planar_B.b_kstr_h = -1;
    }
  }

  switch (dynsim_2link_planar_B.b_kstr_h) {
   case 0:
    memset(&dynsim_2link_planar_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.TJ_c[0] = 1.0;
    dynsim_2link_planar_B.TJ_c[5] = 1.0;
    dynsim_2link_planar_B.TJ_c[10] = 1.0;
    dynsim_2link_planar_B.TJ_c[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v_l);
    dynsim_2link_planar_B.axang_idx_0 = dynsim_2link_planar_B.v_l[0];
    dynsim_2link_planar_B.axang_idx_1 = dynsim_2link_planar_B.v_l[1];
    dynsim_2link_planar_B.axang_idx_2 = dynsim_2link_planar_B.v_l[2];
    dynsim_2link_planar_B.b_b = 1.0 / sqrt((dynsim_2link_planar_B.axang_idx_0 *
      dynsim_2link_planar_B.axang_idx_0 + dynsim_2link_planar_B.axang_idx_1 *
      dynsim_2link_planar_B.axang_idx_1) + dynsim_2link_planar_B.axang_idx_2 *
      dynsim_2link_planar_B.axang_idx_2);
    dynsim_2link_planar_B.v_l[0] = dynsim_2link_planar_B.axang_idx_0 *
      dynsim_2link_planar_B.b_b;
    dynsim_2link_planar_B.v_l[1] = dynsim_2link_planar_B.axang_idx_1 *
      dynsim_2link_planar_B.b_b;
    dynsim_2link_planar_B.v_l[2] = dynsim_2link_planar_B.axang_idx_2 *
      dynsim_2link_planar_B.b_b;
    dynsim_2link_planar_B.axang_idx_0 = dynsim_2link_planar_B.v_l[1] *
      dynsim_2link_planar_B.v_l[0] * 0.0;
    dynsim_2link_planar_B.axang_idx_1 = dynsim_2link_planar_B.v_l[2] *
      dynsim_2link_planar_B.v_l[0] * 0.0;
    dynsim_2link_planar_B.axang_idx_2 = dynsim_2link_planar_B.v_l[2] *
      dynsim_2link_planar_B.v_l[1] * 0.0;
    dynsim_2link_planar_cat(dynsim_2link_planar_B.v_l[0] *
      dynsim_2link_planar_B.v_l[0] * 0.0 + 1.0,
      dynsim_2link_planar_B.axang_idx_0 - dynsim_2link_planar_B.v_l[2] * 0.0,
      dynsim_2link_planar_B.axang_idx_1 + dynsim_2link_planar_B.v_l[1] * 0.0,
      dynsim_2link_planar_B.axang_idx_0 + dynsim_2link_planar_B.v_l[2] * 0.0,
      dynsim_2link_planar_B.v_l[1] * dynsim_2link_planar_B.v_l[1] * 0.0 + 1.0,
      dynsim_2link_planar_B.axang_idx_2 - dynsim_2link_planar_B.v_l[0] * 0.0,
      dynsim_2link_planar_B.axang_idx_1 - dynsim_2link_planar_B.v_l[1] * 0.0,
      dynsim_2link_planar_B.axang_idx_2 + dynsim_2link_planar_B.v_l[0] * 0.0,
      dynsim_2link_planar_B.v_l[2] * dynsim_2link_planar_B.v_l[2] * 0.0 + 1.0,
      dynsim_2link_planar_B.tempR_f);
    for (dynsim_2link_planar_B.b_kstr_h = 0; dynsim_2link_planar_B.b_kstr_h < 3;
         dynsim_2link_planar_B.b_kstr_h++) {
      dynsim_2link_planar_B.kstr_j = dynsim_2link_planar_B.b_kstr_h + 1;
      dynsim_2link_planar_B.R_cv[dynsim_2link_planar_B.kstr_j - 1] =
        dynsim_2link_planar_B.tempR_f[(dynsim_2link_planar_B.kstr_j - 1) * 3];
      dynsim_2link_planar_B.kstr_j = dynsim_2link_planar_B.b_kstr_h + 1;
      dynsim_2link_planar_B.R_cv[dynsim_2link_planar_B.kstr_j + 2] =
        dynsim_2link_planar_B.tempR_f[(dynsim_2link_planar_B.kstr_j - 1) * 3 + 1];
      dynsim_2link_planar_B.kstr_j = dynsim_2link_planar_B.b_kstr_h + 1;
      dynsim_2link_planar_B.R_cv[dynsim_2link_planar_B.kstr_j + 5] =
        dynsim_2link_planar_B.tempR_f[(dynsim_2link_planar_B.kstr_j - 1) * 3 + 2];
    }

    memset(&dynsim_2link_planar_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    for (dynsim_2link_planar_B.b_kstr_h = 0; dynsim_2link_planar_B.b_kstr_h < 3;
         dynsim_2link_planar_B.b_kstr_h++) {
      dynsim_2link_planar_B.kstr_j = dynsim_2link_planar_B.b_kstr_h << 2;
      dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.kstr_j] =
        dynsim_2link_planar_B.R_cv[3 * dynsim_2link_planar_B.b_kstr_h];
      dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.kstr_j + 1] =
        dynsim_2link_planar_B.R_cv[3 * dynsim_2link_planar_B.b_kstr_h + 1];
      dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.kstr_j + 2] =
        dynsim_2link_planar_B.R_cv[3 * dynsim_2link_planar_B.b_kstr_h + 2];
    }

    dynsim_2link_planar_B.TJ_c[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v_l);
    memset(&dynsim_2link_planar_B.tempR_f[0], 0, 9U * sizeof(real_T));
    dynsim_2link_planar_B.tempR_f[0] = 1.0;
    dynsim_2link_planar_B.tempR_f[4] = 1.0;
    dynsim_2link_planar_B.tempR_f[8] = 1.0;
    for (dynsim_2link_planar_B.b_kstr_h = 0; dynsim_2link_planar_B.b_kstr_h < 3;
         dynsim_2link_planar_B.b_kstr_h++) {
      dynsim_2link_planar_B.kstr_j = dynsim_2link_planar_B.b_kstr_h << 2;
      dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.kstr_j] =
        dynsim_2link_planar_B.tempR_f[3 * dynsim_2link_planar_B.b_kstr_h];
      dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.kstr_j + 1] =
        dynsim_2link_planar_B.tempR_f[3 * dynsim_2link_planar_B.b_kstr_h + 1];
      dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.kstr_j + 2] =
        dynsim_2link_planar_B.tempR_f[3 * dynsim_2link_planar_B.b_kstr_h + 2];
      dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.b_kstr_h + 12] =
        dynsim_2link_planar_B.v_l[dynsim_2link_planar_B.b_kstr_h] * 0.0;
    }

    dynsim_2link_planar_B.TJ_c[3] = 0.0;
    dynsim_2link_planar_B.TJ_c[7] = 0.0;
    dynsim_2link_planar_B.TJ_c[11] = 0.0;
    dynsim_2link_planar_B.TJ_c[15] = 1.0;
    break;
  }

  for (dynsim_2link_planar_B.b_kstr_h = 0; dynsim_2link_planar_B.b_kstr_h < 4;
       dynsim_2link_planar_B.b_kstr_h++) {
    for (dynsim_2link_planar_B.kstr_j = 0; dynsim_2link_planar_B.kstr_j < 4;
         dynsim_2link_planar_B.kstr_j++) {
      dynsim_2link_planar_B.obj_tmp_tmp_c = dynsim_2link_planar_B.kstr_j << 2;
      dynsim_2link_planar_B.obj_tmp_c = dynsim_2link_planar_B.b_kstr_h +
        dynsim_2link_planar_B.obj_tmp_tmp_c;
      dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.obj_tmp_c] = 0.0;
      dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.obj_tmp_c] +=
        dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.obj_tmp_tmp_c] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_h];
      dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.obj_tmp_c] +=
        dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.obj_tmp_tmp_c + 1] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_h + 4];
      dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.obj_tmp_c] +=
        dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.obj_tmp_tmp_c + 2] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_h + 8];
      dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.obj_tmp_c] +=
        dynsim_2link_planar_B.TJ_c[dynsim_2link_planar_B.obj_tmp_tmp_c + 3] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_h + 12];
    }

    for (dynsim_2link_planar_B.kstr_j = 0; dynsim_2link_planar_B.kstr_j < 4;
         dynsim_2link_planar_B.kstr_j++) {
      dynsim_2link_planar_B.obj_tmp_tmp_c = dynsim_2link_planar_B.kstr_j << 2;
      dynsim_2link_planar_B.obj_tmp_c = dynsim_2link_planar_B.b_kstr_h +
        dynsim_2link_planar_B.obj_tmp_tmp_c;
      T[dynsim_2link_planar_B.obj_tmp_c] = 0.0;
      T[dynsim_2link_planar_B.obj_tmp_c] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_c] *
        dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.b_kstr_h];
      T[dynsim_2link_planar_B.obj_tmp_c] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_c + 1] *
        dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.b_kstr_h + 4];
      T[dynsim_2link_planar_B.obj_tmp_c] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_c + 2] *
        dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.b_kstr_h + 8];
      T[dynsim_2link_planar_B.obj_tmp_c] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_c + 3] *
        dynsim_2link_planar_B.obj_k[dynsim_2link_planar_B.b_kstr_h + 12];
    }
  }
}

static void dynsim_2link_planar_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (dynsim_2link_planar_B.i3 = 0; dynsim_2link_planar_B.i3 < 3;
       dynsim_2link_planar_B.i3++) {
    dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.i3] =
      T[dynsim_2link_planar_B.i3];
    dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.i3 + 1] =
      T[dynsim_2link_planar_B.i3 + 4];
    dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.i3 + 2] =
      T[dynsim_2link_planar_B.i3 + 8];
  }

  for (dynsim_2link_planar_B.i3 = 0; dynsim_2link_planar_B.i3 < 9;
       dynsim_2link_planar_B.i3++) {
    dynsim_2link_planar_B.R_g1[dynsim_2link_planar_B.i3] =
      -dynsim_2link_planar_B.R_g[dynsim_2link_planar_B.i3];
  }

  for (dynsim_2link_planar_B.i3 = 0; dynsim_2link_planar_B.i3 < 3;
       dynsim_2link_planar_B.i3++) {
    dynsim_2link_planar_B.Tinv_tmp = dynsim_2link_planar_B.i3 << 2;
    Tinv[dynsim_2link_planar_B.Tinv_tmp] = dynsim_2link_planar_B.R_g[3 *
      dynsim_2link_planar_B.i3];
    Tinv[dynsim_2link_planar_B.Tinv_tmp + 1] = dynsim_2link_planar_B.R_g[3 *
      dynsim_2link_planar_B.i3 + 1];
    Tinv[dynsim_2link_planar_B.Tinv_tmp + 2] = dynsim_2link_planar_B.R_g[3 *
      dynsim_2link_planar_B.i3 + 2];
    Tinv[dynsim_2link_planar_B.i3 + 12] =
      dynsim_2link_planar_B.R_g1[dynsim_2link_planar_B.i3 + 6] * T[14] +
      (dynsim_2link_planar_B.R_g1[dynsim_2link_planar_B.i3 + 3] * T[13] +
       dynsim_2link_planar_B.R_g1[dynsim_2link_planar_B.i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void dynsim_2lin_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  dynsim_2link_planar_B.dv2[0] = 0.0;
  dynsim_2link_planar_B.dv2[3] = -T[14];
  dynsim_2link_planar_B.dv2[6] = T[13];
  dynsim_2link_planar_B.dv2[1] = T[14];
  dynsim_2link_planar_B.dv2[4] = 0.0;
  dynsim_2link_planar_B.dv2[7] = -T[12];
  dynsim_2link_planar_B.dv2[2] = -T[13];
  dynsim_2link_planar_B.dv2[5] = T[12];
  dynsim_2link_planar_B.dv2[8] = 0.0;
  for (dynsim_2link_planar_B.i1 = 0; dynsim_2link_planar_B.i1 < 3;
       dynsim_2link_planar_B.i1++) {
    for (dynsim_2link_planar_B.X_tmp_p = 0; dynsim_2link_planar_B.X_tmp_p < 3;
         dynsim_2link_planar_B.X_tmp_p++) {
      dynsim_2link_planar_B.X_tmp_p5 = dynsim_2link_planar_B.i1 + 3 *
        dynsim_2link_planar_B.X_tmp_p;
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p5] = 0.0;
      dynsim_2link_planar_B.i2 = dynsim_2link_planar_B.X_tmp_p << 2;
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p5] +=
        T[dynsim_2link_planar_B.i2] *
        dynsim_2link_planar_B.dv2[dynsim_2link_planar_B.i1];
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p5] +=
        T[dynsim_2link_planar_B.i2 + 1] *
        dynsim_2link_planar_B.dv2[dynsim_2link_planar_B.i1 + 3];
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p5] +=
        T[dynsim_2link_planar_B.i2 + 2] *
        dynsim_2link_planar_B.dv2[dynsim_2link_planar_B.i1 + 6];
      X[dynsim_2link_planar_B.X_tmp_p + 6 * dynsim_2link_planar_B.i1] = T
        [(dynsim_2link_planar_B.i1 << 2) + dynsim_2link_planar_B.X_tmp_p];
      X[dynsim_2link_planar_B.X_tmp_p + 6 * (dynsim_2link_planar_B.i1 + 3)] =
        0.0;
    }
  }

  for (dynsim_2link_planar_B.i1 = 0; dynsim_2link_planar_B.i1 < 3;
       dynsim_2link_planar_B.i1++) {
    X[6 * dynsim_2link_planar_B.i1 + 3] = dynsim_2link_planar_B.dv3[3 *
      dynsim_2link_planar_B.i1];
    dynsim_2link_planar_B.X_tmp_p = dynsim_2link_planar_B.i1 << 2;
    dynsim_2link_planar_B.X_tmp_p5 = 6 * (dynsim_2link_planar_B.i1 + 3);
    X[dynsim_2link_planar_B.X_tmp_p5 + 3] = T[dynsim_2link_planar_B.X_tmp_p];
    X[6 * dynsim_2link_planar_B.i1 + 4] = dynsim_2link_planar_B.dv3[3 *
      dynsim_2link_planar_B.i1 + 1];
    X[dynsim_2link_planar_B.X_tmp_p5 + 4] = T[dynsim_2link_planar_B.X_tmp_p + 1];
    X[6 * dynsim_2link_planar_B.i1 + 5] = dynsim_2link_planar_B.dv3[3 *
      dynsim_2link_planar_B.i1 + 2];
    X[dynsim_2link_planar_B.X_tmp_p5 + 5] = T[dynsim_2link_planar_B.X_tmp_p + 2];
  }
}

static void dynsim_2link_pla_emxInit_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_dynsim_2link__T *emxArray;
  *pEmxArray = (emxArray_char_T_dynsim_2link__T *)malloc(sizeof
    (emxArray_char_T_dynsim_2link__T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_2link_planar_B.i_l = 0; dynsim_2link_planar_B.i_l < numDimensions;
       dynsim_2link_planar_B.i_l++) {
    emxArray->size[dynsim_2link_planar_B.i_l] = 0;
  }
}

static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_2link__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_2link_planar_B.newNumel_e = 1;
  for (dynsim_2link_planar_B.i_ax = 0; dynsim_2link_planar_B.i_ax <
       emxArray->numDimensions; dynsim_2link_planar_B.i_ax++) {
    dynsim_2link_planar_B.newNumel_e *= emxArray->
      size[dynsim_2link_planar_B.i_ax];
  }

  if (dynsim_2link_planar_B.newNumel_e > emxArray->allocatedSize) {
    dynsim_2link_planar_B.i_ax = emxArray->allocatedSize;
    if (dynsim_2link_planar_B.i_ax < 16) {
      dynsim_2link_planar_B.i_ax = 16;
    }

    while (dynsim_2link_planar_B.i_ax < dynsim_2link_planar_B.newNumel_e) {
      if (dynsim_2link_planar_B.i_ax > 1073741823) {
        dynsim_2link_planar_B.i_ax = MAX_int32_T;
      } else {
        dynsim_2link_planar_B.i_ax <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_2link_planar_B.i_ax), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = dynsim_2link_planar_B.i_ax;
    emxArray->canFreeData = true;
  }
}

static void dynsim_2link_pla_emxFree_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_dynsim_2link__T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_dynsim_2link__T *)NULL;
  }
}

static void dynsim_2link_pla_emxFree_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_dynsim_2link__T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_dynsim_2link__T *)NULL;
  }
}

static void dynsim_2lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynsim_2_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynsim_2link_plan_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynsim_2_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMatri(n_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], emxArray_real_T_dynsim_2link__T *H,
  emxArray_real_T_dynsim_2link__T *lambda)
{
  emxArray_f_cell_wrap_dynsim_2_T *Ic;
  emxArray_f_cell_wrap_dynsim_2_T *X;
  emxArray_real_T_dynsim_2link__T *lambda_;
  emxArray_real_T_dynsim_2link__T *Si;
  emxArray_real_T_dynsim_2link__T *Fi;
  emxArray_real_T_dynsim_2link__T *Hji;
  emxArray_real_T_dynsim_2link__T *s;
  l_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_dynsim_2link__T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  dynsim_2link_planar_B.nb_h = robot->NumBodies;
  dynsim_2link_planar_B.vNum_b = robot->VelocityNumber;
  dynsim_2link_planar_B.f_n = H->size[0] * H->size[1];
  dynsim_2link_planar_B.b_i = static_cast<int32_T>(dynsim_2link_planar_B.vNum_b);
  H->size[0] = dynsim_2link_planar_B.b_i;
  H->size[1] = dynsim_2link_planar_B.b_i;
  dynsim_emxEnsureCapacity_real_T(H, dynsim_2link_planar_B.f_n);
  dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.b_i *
    dynsim_2link_planar_B.b_i - 1;
  for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
       dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
    H->data[dynsim_2link_planar_B.f_n] = 0.0;
  }

  dynsim_2link_pla_emxInit_real_T(&lambda_, 2);
  dynsim_2link_planar_B.f_n = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>(dynsim_2link_planar_B.nb_h);
  lambda_->size[1] = dynsim_2link_planar_B.nm1d2;
  dynsim_emxEnsureCapacity_real_T(lambda_, dynsim_2link_planar_B.f_n);
  dynsim_2link_planar_B.idx = dynsim_2link_planar_B.nm1d2 - 1;
  for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
       dynsim_2link_planar_B.idx; dynsim_2link_planar_B.f_n++) {
    lambda_->data[dynsim_2link_planar_B.f_n] = 0.0;
  }

  dynsim_2link_planar_B.f_n = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = dynsim_2link_planar_B.b_i;
  dynsim_emxEnsureCapacity_real_T(lambda, dynsim_2link_planar_B.f_n);
  dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.b_i - 1;
  for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
       dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
    lambda->data[dynsim_2link_planar_B.f_n] = 0.0;
  }

  dynsim_2lin_emxInit_f_cell_wrap(&Ic, 2);
  dynsim_2lin_emxInit_f_cell_wrap(&X, 2);
  dynsim_2link_planar_B.f_n = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = dynsim_2link_planar_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(Ic, dynsim_2link_planar_B.f_n);
  dynsim_2link_planar_B.f_n = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_2link_planar_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(X, dynsim_2link_planar_B.f_n);
  for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i <=
       dynsim_2link_planar_B.idx; dynsim_2link_planar_B.b_i++) {
    for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 36;
         dynsim_2link_planar_B.f_n++) {
      Ic->data[dynsim_2link_planar_B.b_i].f1[dynsim_2link_planar_B.f_n] =
        robot->Bodies[dynsim_2link_planar_B.b_i]->
        SpatialInertia[dynsim_2link_planar_B.f_n];
    }

    dynsim_2link_planar_B.vNum_b = robot->
      PositionDoFMap[dynsim_2link_planar_B.b_i];
    dynsim_2link_planar_B.p_idx_1 = robot->
      PositionDoFMap[dynsim_2link_planar_B.b_i + 6];
    if (dynsim_2link_planar_B.p_idx_1 < dynsim_2link_planar_B.vNum_b) {
      obj = robot->Bodies[dynsim_2link_planar_B.b_i];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynsim_2link_planar_B.T_m);
    } else {
      if (dynsim_2link_planar_B.vNum_b > dynsim_2link_planar_B.p_idx_1) {
        dynsim_2link_planar_B.nm1d2 = 0;
        dynsim_2link_planar_B.f_n = -1;
      } else {
        dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>
          (dynsim_2link_planar_B.vNum_b) - 1;
        dynsim_2link_planar_B.f_n = static_cast<int32_T>
          (dynsim_2link_planar_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[dynsim_2link_planar_B.b_i];
      dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.f_n -
        dynsim_2link_planar_B.nm1d2;
      dynsim_2link_planar_B.q_size_m = dynsim_2link_planar_B.loop_ub + 1;
      for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
           dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
        dynsim_2link_planar_B.q_data_p[dynsim_2link_planar_B.f_n] =
          q[dynsim_2link_planar_B.nm1d2 + dynsim_2link_planar_B.f_n];
      }

      rigidBodyJoint_transformBodyT_a(&obj->JointInternal,
        dynsim_2link_planar_B.q_data_p, &dynsim_2link_planar_B.q_size_m,
        dynsim_2link_planar_B.T_m);
    }

    dynsim_2link_planar_tforminv(dynsim_2link_planar_B.T_m,
      dynsim_2link_planar_B.dv);
    dynsim_2lin_tformToSpatialXform(dynsim_2link_planar_B.dv, X->
      data[dynsim_2link_planar_B.b_i].f1);
  }

  dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>(((-1.0 -
    dynsim_2link_planar_B.nb_h) + 1.0) / -1.0) - 1;
  dynsim_2link_pla_emxInit_real_T(&Si, 2);
  dynsim_2link_pla_emxInit_real_T(&Fi, 2);
  dynsim_2link_pla_emxInit_real_T(&Hji, 2);
  dynsim_2link_pla_emxInit_char_T(&a, 2);
  for (dynsim_2link_planar_B.idx = 0; dynsim_2link_planar_B.idx <=
       dynsim_2link_planar_B.nm1d2; dynsim_2link_planar_B.idx++) {
    dynsim_2link_planar_B.n_m = static_cast<int32_T>(dynsim_2link_planar_B.nb_h
      + -static_cast<real_T>(dynsim_2link_planar_B.idx));
    dynsim_2link_planar_B.pid_tmp = dynsim_2link_planar_B.n_m - 1;
    dynsim_2link_planar_B.pid = robot->Bodies[dynsim_2link_planar_B.pid_tmp]
      ->ParentIndex;
    dynsim_2link_planar_B.vNum_b = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_m - 1];
    dynsim_2link_planar_B.p_idx_1 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_m + 5];
    if (dynsim_2link_planar_B.pid > 0.0) {
      for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
           dynsim_2link_planar_B.f_n++) {
        for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i < 6;
             dynsim_2link_planar_B.b_i++) {
          dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_n + 6 *
            dynsim_2link_planar_B.b_i;
          dynsim_2link_planar_B.X[dynsim_2link_planar_B.X_tmp] = 0.0;
          for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <
               6; dynsim_2link_planar_B.loop_ub++) {
            dynsim_2link_planar_B.X[dynsim_2link_planar_B.X_tmp] += X->
              data[dynsim_2link_planar_B.pid_tmp].f1[6 *
              dynsim_2link_planar_B.f_n + dynsim_2link_planar_B.loop_ub] *
              Ic->data[dynsim_2link_planar_B.pid_tmp].f1[6 *
              dynsim_2link_planar_B.b_i + dynsim_2link_planar_B.loop_ub];
          }
        }
      }

      for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
           dynsim_2link_planar_B.f_n++) {
        for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i < 6;
             dynsim_2link_planar_B.b_i++) {
          dynsim_2link_planar_B.b_idx_0_d = 0.0;
          for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <
               6; dynsim_2link_planar_B.loop_ub++) {
            dynsim_2link_planar_B.b_idx_0_d += dynsim_2link_planar_B.X[6 *
              dynsim_2link_planar_B.loop_ub + dynsim_2link_planar_B.f_n] *
              X->data[dynsim_2link_planar_B.pid_tmp].f1[6 *
              dynsim_2link_planar_B.b_i + dynsim_2link_planar_B.loop_ub];
          }

          dynsim_2link_planar_B.loop_ub = 6 * dynsim_2link_planar_B.b_i +
            dynsim_2link_planar_B.f_n;
          Ic->data[static_cast<int32_T>(dynsim_2link_planar_B.pid) - 1]
            .f1[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.b_idx_0_d;
        }
      }

      lambda_->data[dynsim_2link_planar_B.pid_tmp] = dynsim_2link_planar_B.pid;
      exitg1 = false;
      while ((!exitg1) && (lambda_->data[dynsim_2link_planar_B.pid_tmp] > 0.0))
      {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[dynsim_2link_planar_B.pid_tmp]) - 1];
        dynsim_2link_planar_B.f_n = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        dynsim_emxEnsureCapacity_char_T(a, dynsim_2link_planar_B.f_n);
        dynsim_2link_planar_B.loop_ub = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
             dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
          a->data[dynsim_2link_planar_B.f_n] = obj->JointInternal.Type->
            data[dynsim_2link_planar_B.f_n];
        }

        for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 5;
             dynsim_2link_planar_B.f_n++) {
          dynsim_2link_planar_B.b_a[dynsim_2link_planar_B.f_n] =
            tmp[dynsim_2link_planar_B.f_n];
        }

        dynsim_2link_planar_B.b_bool_o = false;
        if (a->size[1] == 5) {
          dynsim_2link_planar_B.f_n = 1;
          do {
            exitg2 = 0;
            if (dynsim_2link_planar_B.f_n - 1 < 5) {
              dynsim_2link_planar_B.b_i = dynsim_2link_planar_B.f_n - 1;
              if (a->data[dynsim_2link_planar_B.b_i] !=
                  dynsim_2link_planar_B.b_a[dynsim_2link_planar_B.b_i]) {
                exitg2 = 1;
              } else {
                dynsim_2link_planar_B.f_n++;
              }
            } else {
              dynsim_2link_planar_B.b_bool_o = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (dynsim_2link_planar_B.b_bool_o) {
          lambda_->data[dynsim_2link_planar_B.pid_tmp] = robot->Bodies[
            static_cast<int32_T>(lambda_->data[dynsim_2link_planar_B.pid_tmp]) -
            1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    dynsim_2link_planar_B.b_idx_0_d = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_m - 1];
    dynsim_2link_planar_B.b_idx_1_e = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_m + 5];
    if (dynsim_2link_planar_B.b_idx_0_d <= dynsim_2link_planar_B.b_idx_1_e) {
      obj = robot->Bodies[dynsim_2link_planar_B.pid_tmp];
      dynsim_2link_planar_B.f_n = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_n);
      dynsim_2link_planar_B.loop_ub = obj->JointInternal.MotionSubspace->size[0]
        * obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
           dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
        Si->data[dynsim_2link_planar_B.f_n] = obj->
          JointInternal.MotionSubspace->data[dynsim_2link_planar_B.f_n];
      }

      dynsim_2link_planar_B.n_m = Si->size[1] - 1;
      dynsim_2link_planar_B.f_n = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_2link_planar_B.f_n);
      for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <=
           dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.loop_ub++) {
        dynsim_2link_planar_B.coffset_tmp = dynsim_2link_planar_B.loop_ub * 6 -
          1;
        for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i < 6;
             dynsim_2link_planar_B.b_i++) {
          dynsim_2link_planar_B.s = 0.0;
          for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
               dynsim_2link_planar_B.f_n++) {
            dynsim_2link_planar_B.s += Ic->data[dynsim_2link_planar_B.pid_tmp].
              f1[dynsim_2link_planar_B.f_n * 6 + dynsim_2link_planar_B.b_i] *
              Si->data[(dynsim_2link_planar_B.coffset_tmp +
                        dynsim_2link_planar_B.f_n) + 1];
          }

          Fi->data[(dynsim_2link_planar_B.coffset_tmp +
                    dynsim_2link_planar_B.b_i) + 1] = dynsim_2link_planar_B.s;
        }
      }

      if (dynsim_2link_planar_B.vNum_b > dynsim_2link_planar_B.p_idx_1) {
        dynsim_2link_planar_B.coffset_tmp = 0;
        dynsim_2link_planar_B.cb = 0;
      } else {
        dynsim_2link_planar_B.coffset_tmp = static_cast<int32_T>
          (dynsim_2link_planar_B.vNum_b) - 1;
        dynsim_2link_planar_B.cb = static_cast<int32_T>
          (dynsim_2link_planar_B.vNum_b) - 1;
      }

      dynsim_2link_planar_B.m_c = Si->size[1];
      dynsim_2link_planar_B.n_m = Fi->size[1] - 1;
      dynsim_2link_planar_B.f_n = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Hji, dynsim_2link_planar_B.f_n);
      for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <=
           dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.loop_ub++) {
        dynsim_2link_planar_B.coffset = dynsim_2link_planar_B.loop_ub *
          dynsim_2link_planar_B.m_c - 1;
        dynsim_2link_planar_B.boffset = dynsim_2link_planar_B.loop_ub * 6 - 1;
        for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i <
             dynsim_2link_planar_B.m_c; dynsim_2link_planar_B.b_i++) {
          dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i * 6 - 1;
          dynsim_2link_planar_B.s = 0.0;
          for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
               dynsim_2link_planar_B.f_n++) {
            dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_n + 1;
            dynsim_2link_planar_B.s += Si->data[dynsim_2link_planar_B.aoffset_m
              + dynsim_2link_planar_B.X_tmp] * Fi->
              data[dynsim_2link_planar_B.boffset + dynsim_2link_planar_B.X_tmp];
          }

          Hji->data[(dynsim_2link_planar_B.coffset + dynsim_2link_planar_B.b_i)
            + 1] = dynsim_2link_planar_B.s;
        }
      }

      dynsim_2link_planar_B.loop_ub = Hji->size[1];
      for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <
           dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
        dynsim_2link_planar_B.n_m = Hji->size[0];
        for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i <
             dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.b_i++) {
          H->data[(dynsim_2link_planar_B.coffset_tmp + dynsim_2link_planar_B.b_i)
            + H->size[0] * (dynsim_2link_planar_B.cb + dynsim_2link_planar_B.f_n)]
            = Hji->data[Hji->size[0] * dynsim_2link_planar_B.f_n +
            dynsim_2link_planar_B.b_i];
        }
      }

      dynsim_2link_planar_B.n_m = Fi->size[1];
      dynsim_2link_planar_B.f_n = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_n);
      dynsim_2link_planar_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
      for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
           dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
        Si->data[dynsim_2link_planar_B.f_n] = Fi->data[dynsim_2link_planar_B.f_n];
      }

      dynsim_2link_planar_B.f_n = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = dynsim_2link_planar_B.n_m;
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_2link_planar_B.f_n);
      for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <
           dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.loop_ub++) {
        dynsim_2link_planar_B.coffset_tmp = dynsim_2link_planar_B.loop_ub * 6 -
          1;
        for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i < 6;
             dynsim_2link_planar_B.b_i++) {
          dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i * 6 - 1;
          dynsim_2link_planar_B.s = 0.0;
          for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
               dynsim_2link_planar_B.f_n++) {
            dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_n + 1;
            dynsim_2link_planar_B.s += X->data[dynsim_2link_planar_B.pid_tmp]
              .f1[dynsim_2link_planar_B.aoffset_m + dynsim_2link_planar_B.X_tmp]
              * Si->data[dynsim_2link_planar_B.coffset_tmp +
              dynsim_2link_planar_B.X_tmp];
          }

          Fi->data[(dynsim_2link_planar_B.coffset_tmp +
                    dynsim_2link_planar_B.b_i) + 1] = dynsim_2link_planar_B.s;
        }
      }

      while (dynsim_2link_planar_B.pid > 0.0) {
        dynsim_2link_planar_B.b_i = static_cast<int32_T>
          (dynsim_2link_planar_B.pid);
        dynsim_2link_planar_B.pid_tmp = dynsim_2link_planar_B.b_i - 1;
        obj = robot->Bodies[dynsim_2link_planar_B.pid_tmp];
        dynsim_2link_planar_B.f_n = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_n);
        dynsim_2link_planar_B.loop_ub = obj->JointInternal.MotionSubspace->size
          [0] * obj->JointInternal.MotionSubspace->size[1] - 1;
        for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
             dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
          Si->data[dynsim_2link_planar_B.f_n] =
            obj->JointInternal.MotionSubspace->data[dynsim_2link_planar_B.f_n];
        }

        dynsim_2link_planar_B.b_idx_0_d = robot->
          VelocityDoFMap[dynsim_2link_planar_B.b_i - 1];
        dynsim_2link_planar_B.b_idx_1_e = robot->
          VelocityDoFMap[dynsim_2link_planar_B.b_i + 5];
        if (dynsim_2link_planar_B.b_idx_0_d <= dynsim_2link_planar_B.b_idx_1_e)
        {
          dynsim_2link_planar_B.m_c = Si->size[1];
          dynsim_2link_planar_B.n_m = Fi->size[1] - 1;
          dynsim_2link_planar_B.f_n = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          dynsim_emxEnsureCapacity_real_T(Hji, dynsim_2link_planar_B.f_n);
          for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <=
               dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.loop_ub++) {
            dynsim_2link_planar_B.coffset = dynsim_2link_planar_B.loop_ub *
              dynsim_2link_planar_B.m_c - 1;
            dynsim_2link_planar_B.boffset = dynsim_2link_planar_B.loop_ub * 6 -
              1;
            for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i <
                 dynsim_2link_planar_B.m_c; dynsim_2link_planar_B.b_i++) {
              dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i * 6 -
                1;
              dynsim_2link_planar_B.s = 0.0;
              for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
                   dynsim_2link_planar_B.f_n++) {
                dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_n + 1;
                dynsim_2link_planar_B.s += Si->
                  data[dynsim_2link_planar_B.aoffset_m +
                  dynsim_2link_planar_B.X_tmp] * Fi->
                  data[dynsim_2link_planar_B.boffset +
                  dynsim_2link_planar_B.X_tmp];
              }

              Hji->data[(dynsim_2link_planar_B.coffset +
                         dynsim_2link_planar_B.b_i) + 1] =
                dynsim_2link_planar_B.s;
            }
          }

          if (dynsim_2link_planar_B.b_idx_0_d > dynsim_2link_planar_B.b_idx_1_e)
          {
            dynsim_2link_planar_B.X_tmp = 0;
          } else {
            dynsim_2link_planar_B.X_tmp = static_cast<int32_T>
              (dynsim_2link_planar_B.b_idx_0_d) - 1;
          }

          if (dynsim_2link_planar_B.vNum_b > dynsim_2link_planar_B.p_idx_1) {
            dynsim_2link_planar_B.coffset_tmp = 0;
          } else {
            dynsim_2link_planar_B.coffset_tmp = static_cast<int32_T>
              (dynsim_2link_planar_B.vNum_b) - 1;
          }

          dynsim_2link_planar_B.loop_ub = Hji->size[1];
          for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <
               dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
            dynsim_2link_planar_B.n_m = Hji->size[0];
            for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i <
                 dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.b_i++) {
              H->data[(dynsim_2link_planar_B.X_tmp + dynsim_2link_planar_B.b_i)
                + H->size[0] * (dynsim_2link_planar_B.coffset_tmp +
                                dynsim_2link_planar_B.f_n)] = Hji->data
                [Hji->size[0] * dynsim_2link_planar_B.f_n +
                dynsim_2link_planar_B.b_i];
            }
          }

          if (dynsim_2link_planar_B.vNum_b > dynsim_2link_planar_B.p_idx_1) {
            dynsim_2link_planar_B.X_tmp = 0;
          } else {
            dynsim_2link_planar_B.X_tmp = static_cast<int32_T>
              (dynsim_2link_planar_B.vNum_b) - 1;
          }

          if (dynsim_2link_planar_B.b_idx_0_d > dynsim_2link_planar_B.b_idx_1_e)
          {
            dynsim_2link_planar_B.coffset_tmp = 0;
          } else {
            dynsim_2link_planar_B.coffset_tmp = static_cast<int32_T>
              (dynsim_2link_planar_B.b_idx_0_d) - 1;
          }

          dynsim_2link_planar_B.loop_ub = Hji->size[0];
          for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <
               dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
            dynsim_2link_planar_B.n_m = Hji->size[1];
            for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i <
                 dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.b_i++) {
              H->data[(dynsim_2link_planar_B.X_tmp + dynsim_2link_planar_B.b_i)
                + H->size[0] * (dynsim_2link_planar_B.coffset_tmp +
                                dynsim_2link_planar_B.f_n)] = Hji->data
                [Hji->size[0] * dynsim_2link_planar_B.b_i +
                dynsim_2link_planar_B.f_n];
            }
          }
        }

        dynsim_2link_planar_B.n_m = Fi->size[1];
        dynsim_2link_planar_B.f_n = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_n);
        dynsim_2link_planar_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
        for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
             dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
          Si->data[dynsim_2link_planar_B.f_n] = Fi->
            data[dynsim_2link_planar_B.f_n];
        }

        dynsim_2link_planar_B.f_n = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = dynsim_2link_planar_B.n_m;
        dynsim_emxEnsureCapacity_real_T(Fi, dynsim_2link_planar_B.f_n);
        for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <
             dynsim_2link_planar_B.n_m; dynsim_2link_planar_B.loop_ub++) {
          dynsim_2link_planar_B.coffset_tmp = dynsim_2link_planar_B.loop_ub * 6
            - 1;
          for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i < 6;
               dynsim_2link_planar_B.b_i++) {
            dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i * 6 - 1;
            dynsim_2link_planar_B.s = 0.0;
            for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
                 dynsim_2link_planar_B.f_n++) {
              dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_n + 1;
              dynsim_2link_planar_B.s += X->data[dynsim_2link_planar_B.pid_tmp].
                f1[dynsim_2link_planar_B.aoffset_m + dynsim_2link_planar_B.X_tmp]
                * Si->data[dynsim_2link_planar_B.coffset_tmp +
                dynsim_2link_planar_B.X_tmp];
            }

            Fi->data[(dynsim_2link_planar_B.coffset_tmp +
                      dynsim_2link_planar_B.b_i) + 1] = dynsim_2link_planar_B.s;
          }
        }

        dynsim_2link_planar_B.pid = robot->Bodies[dynsim_2link_planar_B.pid_tmp
          ]->ParentIndex;
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&a);
  dynsim_2link_pla_emxFree_real_T(&Hji);
  dynsim_2link_pla_emxFree_real_T(&Fi);
  dynsim_2link_pla_emxFree_real_T(&Si);
  dynsim_2lin_emxFree_f_cell_wrap(&X);
  dynsim_2lin_emxFree_f_cell_wrap(&Ic);
  for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n < 6;
       dynsim_2link_planar_B.f_n++) {
    dynsim_2link_planar_B.mask[dynsim_2link_planar_B.f_n] =
      (robot->VelocityDoFMap[dynsim_2link_planar_B.f_n] <= robot->
       VelocityDoFMap[dynsim_2link_planar_B.f_n + 6]);
  }

  dynsim_2link_planar_B.idx = 0;
  dynsim_2link_planar_B.f_n = 1;
  exitg1 = false;
  while ((!exitg1) && (dynsim_2link_planar_B.f_n - 1 < 6)) {
    if (dynsim_2link_planar_B.mask[dynsim_2link_planar_B.f_n - 1]) {
      dynsim_2link_planar_B.idx++;
      dynsim_2link_planar_B.ii_data[dynsim_2link_planar_B.idx - 1] =
        dynsim_2link_planar_B.f_n;
      if (dynsim_2link_planar_B.idx >= 6) {
        exitg1 = true;
      } else {
        dynsim_2link_planar_B.f_n++;
      }
    } else {
      dynsim_2link_planar_B.f_n++;
    }
  }

  if (1 > dynsim_2link_planar_B.idx) {
    dynsim_2link_planar_B.idx = 0;
  }

  for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <
       dynsim_2link_planar_B.idx; dynsim_2link_planar_B.f_n++) {
    dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.f_n] =
      dynsim_2link_planar_B.ii_data[dynsim_2link_planar_B.f_n];
  }

  dynsim_2link_planar_B.b_i = dynsim_2link_planar_B.idx - 1;
  dynsim_2link_pla_emxInit_real_T(&s, 2);
  for (dynsim_2link_planar_B.idx = 0; dynsim_2link_planar_B.idx <=
       dynsim_2link_planar_B.b_i; dynsim_2link_planar_B.idx++) {
    dynsim_2link_planar_B.vNum_b = robot->
      VelocityDoFMap[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
      - 1];
    dynsim_2link_planar_B.p_idx_1 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
      + 5];
    if (rtIsNaN(dynsim_2link_planar_B.vNum_b) || rtIsNaN
        (dynsim_2link_planar_B.p_idx_1)) {
      dynsim_2link_planar_B.f_n = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_n);
      s->data[0] = (rtNaN);
    } else if (dynsim_2link_planar_B.p_idx_1 < dynsim_2link_planar_B.vNum_b) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(dynsim_2link_planar_B.vNum_b) || rtIsInf
                (dynsim_2link_planar_B.p_idx_1)) &&
               (dynsim_2link_planar_B.vNum_b == dynsim_2link_planar_B.p_idx_1))
    {
      dynsim_2link_planar_B.f_n = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_n);
      s->data[0] = (rtNaN);
    } else if (floor(dynsim_2link_planar_B.vNum_b) ==
               dynsim_2link_planar_B.vNum_b) {
      dynsim_2link_planar_B.f_n = s->size[0] * s->size[1];
      s->size[0] = 1;
      dynsim_2link_planar_B.loop_ub = static_cast<int32_T>(floor
        (dynsim_2link_planar_B.p_idx_1 - dynsim_2link_planar_B.vNum_b));
      s->size[1] = dynsim_2link_planar_B.loop_ub + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_n);
      for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
           dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
        s->data[dynsim_2link_planar_B.f_n] = dynsim_2link_planar_B.vNum_b +
          static_cast<real_T>(dynsim_2link_planar_B.f_n);
      }
    } else {
      dynsim_2link_planar_B.nb_h = floor((dynsim_2link_planar_B.p_idx_1 -
        dynsim_2link_planar_B.vNum_b) + 0.5);
      dynsim_2link_planar_B.pid = dynsim_2link_planar_B.vNum_b +
        dynsim_2link_planar_B.nb_h;
      dynsim_2link_planar_B.b_idx_0_d = dynsim_2link_planar_B.pid -
        dynsim_2link_planar_B.p_idx_1;
      dynsim_2link_planar_B.b_idx_1_e = fabs(dynsim_2link_planar_B.vNum_b);
      dynsim_2link_planar_B.s = fabs(dynsim_2link_planar_B.p_idx_1);
      if ((dynsim_2link_planar_B.b_idx_1_e > dynsim_2link_planar_B.s) || rtIsNaN
          (dynsim_2link_planar_B.s)) {
        dynsim_2link_planar_B.s = dynsim_2link_planar_B.b_idx_1_e;
      }

      if (fabs(dynsim_2link_planar_B.b_idx_0_d) < 4.4408920985006262E-16 *
          dynsim_2link_planar_B.s) {
        dynsim_2link_planar_B.nb_h++;
        dynsim_2link_planar_B.pid = dynsim_2link_planar_B.p_idx_1;
      } else if (dynsim_2link_planar_B.b_idx_0_d > 0.0) {
        dynsim_2link_planar_B.pid = (dynsim_2link_planar_B.nb_h - 1.0) +
          dynsim_2link_planar_B.vNum_b;
      } else {
        dynsim_2link_planar_B.nb_h++;
      }

      if (dynsim_2link_planar_B.nb_h >= 0.0) {
        dynsim_2link_planar_B.f_n = static_cast<int32_T>
          (dynsim_2link_planar_B.nb_h);
      } else {
        dynsim_2link_planar_B.f_n = 0;
      }

      dynsim_2link_planar_B.n_m = dynsim_2link_planar_B.f_n - 1;
      dynsim_2link_planar_B.f_n = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = dynsim_2link_planar_B.n_m + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_n);
      if (dynsim_2link_planar_B.n_m + 1 > 0) {
        s->data[0] = dynsim_2link_planar_B.vNum_b;
        if (dynsim_2link_planar_B.n_m + 1 > 1) {
          s->data[dynsim_2link_planar_B.n_m] = dynsim_2link_planar_B.pid;
          dynsim_2link_planar_B.nm1d2 = ((dynsim_2link_planar_B.n_m < 0) +
            dynsim_2link_planar_B.n_m) >> 1;
          dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.nm1d2 - 2;
          for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <=
               dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
            dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_n + 1;
            s->data[dynsim_2link_planar_B.X_tmp] = dynsim_2link_planar_B.vNum_b
              + static_cast<real_T>(dynsim_2link_planar_B.X_tmp);
            s->data[dynsim_2link_planar_B.n_m - dynsim_2link_planar_B.X_tmp] =
              dynsim_2link_planar_B.pid - static_cast<real_T>
              (dynsim_2link_planar_B.X_tmp);
          }

          if (dynsim_2link_planar_B.nm1d2 << 1 == dynsim_2link_planar_B.n_m) {
            s->data[dynsim_2link_planar_B.nm1d2] = (dynsim_2link_planar_B.vNum_b
              + dynsim_2link_planar_B.pid) / 2.0;
          } else {
            s->data[dynsim_2link_planar_B.nm1d2] = dynsim_2link_planar_B.vNum_b
              + static_cast<real_T>(dynsim_2link_planar_B.nm1d2);
            s->data[dynsim_2link_planar_B.nm1d2 + 1] = dynsim_2link_planar_B.pid
              - static_cast<real_T>(dynsim_2link_planar_B.nm1d2);
          }
        }
      }
    }

    if (dynsim_2link_planar_B.vNum_b > dynsim_2link_planar_B.p_idx_1) {
      dynsim_2link_planar_B.nm1d2 = 0;
    } else {
      dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>
        (dynsim_2link_planar_B.vNum_b) - 1;
    }

    dynsim_2link_planar_B.loop_ub = s->size[1];
    for (dynsim_2link_planar_B.f_n = 0; dynsim_2link_planar_B.f_n <
         dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.f_n++) {
      lambda->data[dynsim_2link_planar_B.nm1d2 + dynsim_2link_planar_B.f_n] =
        s->data[dynsim_2link_planar_B.f_n] - 1.0;
    }

    if (lambda_->
        data[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(dynsim_2link_planar_B.vNum_b) - 1] = 0.0;
    } else {
      dynsim_2link_planar_B.f_n = static_cast<int32_T>(lambda_->
        data[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
        - 1]);
      dynsim_2link_planar_B.b_idx_1_e = robot->
        VelocityDoFMap[dynsim_2link_planar_B.f_n + 5];
      lambda->data[static_cast<int32_T>(dynsim_2link_planar_B.vNum_b) - 1] =
        dynsim_2link_planar_B.b_idx_1_e;
    }
  }

  dynsim_2link_pla_emxFree_real_T(&s);
  dynsim_2link_pla_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(n_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[36], real_T
  tau[6])
{
  emxArray_f_cell_wrap_dynsim_2_T *X;
  emxArray_f_cell_wrap_dynsim_2_T *Xtree;
  emxArray_real_T_dynsim_2link__T *vJ;
  emxArray_real_T_dynsim_2link__T *vB;
  emxArray_real_T_dynsim_2link__T *aB;
  emxArray_real_T_dynsim_2link__T *f;
  emxArray_real_T_dynsim_2link__T *S;
  emxArray_real_T_dynsim_2link__T *taui;
  l_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_dynsim_2link__T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  dynsim_2link_planar_B.a0[0] = 0.0;
  dynsim_2link_planar_B.a0[1] = 0.0;
  dynsim_2link_planar_B.a0[2] = 0.0;
  dynsim_2link_planar_B.a0[3] = -robot->Gravity[0];
  dynsim_2link_planar_B.a0[4] = -robot->Gravity[1];
  dynsim_2link_planar_B.a0[5] = -robot->Gravity[2];
  dynsim_2link_pla_emxInit_real_T(&vJ, 2);
  dynsim_2link_planar_B.nb = robot->NumBodies;
  dynsim_2link_planar_B.i_n = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  dynsim_2link_planar_B.unnamed_idx_1 = static_cast<int32_T>
    (dynsim_2link_planar_B.nb);
  vJ->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vJ, dynsim_2link_planar_B.i_n);
  dynsim_2link_planar_B.loop_ub_tmp = 6 * dynsim_2link_planar_B.unnamed_idx_1 -
    1;
  for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.i_n++) {
    vJ->data[dynsim_2link_planar_B.i_n] = 0.0;
  }

  dynsim_2link_pla_emxInit_real_T(&vB, 2);
  dynsim_2link_planar_B.i_n = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vB, dynsim_2link_planar_B.i_n);
  for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.i_n++) {
    vB->data[dynsim_2link_planar_B.i_n] = 0.0;
  }

  dynsim_2link_pla_emxInit_real_T(&aB, 2);
  dynsim_2link_planar_B.i_n = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(aB, dynsim_2link_planar_B.i_n);
  for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.i_n++) {
    aB->data[dynsim_2link_planar_B.i_n] = 0.0;
  }

  for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
       dynsim_2link_planar_B.i_n++) {
    tau[dynsim_2link_planar_B.i_n] = 0.0;
  }

  dynsim_2lin_emxInit_f_cell_wrap(&X, 2);
  dynsim_2lin_emxInit_f_cell_wrap(&Xtree, 2);
  dynsim_2link_planar_B.loop_ub_tmp = dynsim_2link_planar_B.unnamed_idx_1 - 1;
  dynsim_2link_planar_B.i_n = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(Xtree, dynsim_2link_planar_B.i_n);
  dynsim_2link_planar_B.i_n = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(X, dynsim_2link_planar_B.i_n);
  for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.b_k++) {
    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 36;
         dynsim_2link_planar_B.i_n++) {
      Xtree->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_n] = 0.0;
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
         dynsim_2link_planar_B.i_n++) {
      Xtree->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_n + 6 *
        dynsim_2link_planar_B.i_n] = 1.0;
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 36;
         dynsim_2link_planar_B.i_n++) {
      X->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_n] = 0.0;
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
         dynsim_2link_planar_B.i_n++) {
      X->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_n + 6 *
        dynsim_2link_planar_B.i_n] = 1.0;
    }
  }

  dynsim_2link_pla_emxInit_real_T(&f, 2);
  dynsim_2link_planar_B.i_n = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(f, dynsim_2link_planar_B.i_n);
  dynsim_2link_pla_emxInit_real_T(&S, 2);
  if (0 <= dynsim_2link_planar_B.loop_ub_tmp) {
    dynsim_2link_planar_B.dv1[0] = 0.0;
    dynsim_2link_planar_B.dv1[4] = 0.0;
    dynsim_2link_planar_B.dv1[8] = 0.0;
  }

  for (dynsim_2link_planar_B.unnamed_idx_1 = 0;
       dynsim_2link_planar_B.unnamed_idx_1 <= dynsim_2link_planar_B.loop_ub_tmp;
       dynsim_2link_planar_B.unnamed_idx_1++) {
    obj = robot->Bodies[dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.i_n = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    dynsim_emxEnsureCapacity_real_T(S, dynsim_2link_planar_B.i_n);
    dynsim_2link_planar_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <=
         dynsim_2link_planar_B.b_k; dynsim_2link_planar_B.i_n++) {
      S->data[dynsim_2link_planar_B.i_n] = obj->
        JointInternal.MotionSubspace->data[dynsim_2link_planar_B.i_n];
    }

    dynsim_2link_planar_B.a_idx_0 = robot->
      PositionDoFMap[dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.a_idx_1 = robot->
      PositionDoFMap[dynsim_2link_planar_B.unnamed_idx_1 + 6];
    dynsim_2link_planar_B.b_idx_0 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.b_idx_1 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.unnamed_idx_1 + 6];
    if (dynsim_2link_planar_B.a_idx_1 < dynsim_2link_planar_B.a_idx_0) {
      obj = robot->Bodies[dynsim_2link_planar_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynsim_2link_planar_B.T);
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        vJ->data[dynsim_2link_planar_B.i_n + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (dynsim_2link_planar_B.a_idx_0 > dynsim_2link_planar_B.a_idx_1) {
        dynsim_2link_planar_B.b_k = 0;
        dynsim_2link_planar_B.i_n = -1;
      } else {
        dynsim_2link_planar_B.b_k = static_cast<int32_T>
          (dynsim_2link_planar_B.a_idx_0) - 1;
        dynsim_2link_planar_B.i_n = static_cast<int32_T>
          (dynsim_2link_planar_B.a_idx_1) - 1;
      }

      if (dynsim_2link_planar_B.b_idx_0 > dynsim_2link_planar_B.b_idx_1) {
        dynsim_2link_planar_B.p = -1;
      } else {
        dynsim_2link_planar_B.p = static_cast<int32_T>
          (dynsim_2link_planar_B.b_idx_0) - 2;
      }

      obj = robot->Bodies[dynsim_2link_planar_B.unnamed_idx_1];
      dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_n -
        dynsim_2link_planar_B.b_k;
      dynsim_2link_planar_B.q_size = dynsim_2link_planar_B.q_size_tmp + 1;
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <=
           dynsim_2link_planar_B.q_size_tmp; dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n] =
          q[dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_n];
      }

      rigidBodyJoint_transformBodyT_a(&obj->JointInternal,
        dynsim_2link_planar_B.q_data, &dynsim_2link_planar_B.q_size,
        dynsim_2link_planar_B.T);
      dynsim_2link_planar_B.inner = S->size[1] - 1;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        vJ->data[dynsim_2link_planar_B.b_k + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] = 0.0;
      }

      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
           dynsim_2link_planar_B.inner; dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.b_k * 6 - 1;
        for (dynsim_2link_planar_B.q_size_tmp = 0;
             dynsim_2link_planar_B.q_size_tmp < 6;
             dynsim_2link_planar_B.q_size_tmp++) {
          dynsim_2link_planar_B.i_n = 6 * dynsim_2link_planar_B.unnamed_idx_1 +
            dynsim_2link_planar_B.q_size_tmp;
          vJ->data[dynsim_2link_planar_B.i_n] += S->data
            [(dynsim_2link_planar_B.aoffset + dynsim_2link_planar_B.q_size_tmp)
            + 1] * qdot[(dynsim_2link_planar_B.p + dynsim_2link_planar_B.b_k) +
            1];
        }
      }
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.R_c[3 * dynsim_2link_planar_B.i_n] =
        dynsim_2link_planar_B.T[dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.R_c[3 * dynsim_2link_planar_B.i_n + 1] =
        dynsim_2link_planar_B.T[dynsim_2link_planar_B.i_n + 4];
      dynsim_2link_planar_B.R_c[3 * dynsim_2link_planar_B.i_n + 2] =
        dynsim_2link_planar_B.T[dynsim_2link_planar_B.i_n + 8];
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 9;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.i_n] =
        -dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.i_n];
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_n << 2;
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k] =
        dynsim_2link_planar_B.R_c[3 * dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 1] =
        dynsim_2link_planar_B.R_c[3 * dynsim_2link_planar_B.i_n + 1];
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 2] =
        dynsim_2link_planar_B.R_c[3 * dynsim_2link_planar_B.i_n + 2];
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.i_n + 12] =
        dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.i_n + 6] *
        dynsim_2link_planar_B.T[14] +
        (dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.i_n + 3] *
         dynsim_2link_planar_B.T[13] +
         dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.i_n] *
         dynsim_2link_planar_B.T[12]);
    }

    dynsim_2link_planar_B.Tinv[3] = 0.0;
    dynsim_2link_planar_B.Tinv[7] = 0.0;
    dynsim_2link_planar_B.Tinv[11] = 0.0;
    dynsim_2link_planar_B.Tinv[15] = 1.0;
    dynsim_2link_planar_B.dv1[3] = -dynsim_2link_planar_B.Tinv[14];
    dynsim_2link_planar_B.dv1[6] = dynsim_2link_planar_B.Tinv[13];
    dynsim_2link_planar_B.dv1[1] = dynsim_2link_planar_B.Tinv[14];
    dynsim_2link_planar_B.dv1[7] = -dynsim_2link_planar_B.Tinv[12];
    dynsim_2link_planar_B.dv1[2] = -dynsim_2link_planar_B.Tinv[13];
    dynsim_2link_planar_B.dv1[5] = dynsim_2link_planar_B.Tinv[12];
    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
         dynsim_2link_planar_B.i_n++) {
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 3;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_n + 3 *
          dynsim_2link_planar_B.b_k;
        dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.q_size_tmp] = 0.0;
        dynsim_2link_planar_B.p = dynsim_2link_planar_B.b_k << 2;
        dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.q_size_tmp] +=
          dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.p] *
          dynsim_2link_planar_B.dv1[dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.q_size_tmp] +=
          dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.p + 1] *
          dynsim_2link_planar_B.dv1[dynsim_2link_planar_B.i_n + 3];
        dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.q_size_tmp] +=
          dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.p + 2] *
          dynsim_2link_planar_B.dv1[dynsim_2link_planar_B.i_n + 6];
        X->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.b_k + 6 * dynsim_2link_planar_B.i_n] =
          dynsim_2link_planar_B.Tinv[(dynsim_2link_planar_B.i_n << 2) +
          dynsim_2link_planar_B.b_k];
        X->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.b_k + 6 * (dynsim_2link_planar_B.i_n + 3)] =
          0.0;
      }
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
         dynsim_2link_planar_B.i_n++) {
      X->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_2link_planar_B.i_n + 3] = dynsim_2link_planar_B.R_c[3 *
        dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_n << 2;
      dynsim_2link_planar_B.q_size_tmp = 6 * (dynsim_2link_planar_B.i_n + 3);
      X->data[dynsim_2link_planar_B.unnamed_idx_1]
        .f1[dynsim_2link_planar_B.q_size_tmp + 3] =
        dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k];
      X->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_2link_planar_B.i_n + 4] = dynsim_2link_planar_B.R_c[3 *
        dynsim_2link_planar_B.i_n + 1];
      X->data[dynsim_2link_planar_B.unnamed_idx_1]
        .f1[dynsim_2link_planar_B.q_size_tmp + 4] =
        dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 1];
      X->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_2link_planar_B.i_n + 5] = dynsim_2link_planar_B.R_c[3 *
        dynsim_2link_planar_B.i_n + 2];
      X->data[dynsim_2link_planar_B.unnamed_idx_1]
        .f1[dynsim_2link_planar_B.q_size_tmp + 5] =
        dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 2];
    }

    dynsim_2link_planar_B.a_idx_0 = robot->
      Bodies[dynsim_2link_planar_B.unnamed_idx_1]->ParentIndex;
    if (dynsim_2link_planar_B.a_idx_0 > 0.0) {
      dynsim_2link_planar_B.m = static_cast<int32_T>
        (dynsim_2link_planar_B.a_idx_0);
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += vB->data[(dynsim_2link_planar_B.m - 1)
            * 6 + dynsim_2link_planar_B.b_k] * X->
            data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_n];
        }

        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n] = vJ->data[6 *
          dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.i_n] +
          dynsim_2link_planar_B.a_idx_1;
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        vB->data[dynsim_2link_planar_B.i_n + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] =
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n];
      }

      dynsim_2link_planar_B.inner = S->size[1] - 1;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.b_k] = 0.0;
      }

      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
           dynsim_2link_planar_B.inner; dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.b_k * 6 - 1;
        for (dynsim_2link_planar_B.q_size_tmp = 0;
             dynsim_2link_planar_B.q_size_tmp < 6;
             dynsim_2link_planar_B.q_size_tmp++) {
          dynsim_2link_planar_B.a_idx_1 = S->data[(dynsim_2link_planar_B.aoffset
            + dynsim_2link_planar_B.q_size_tmp) + 1] * 0.0 +
            dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp];
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp] =
            dynsim_2link_planar_B.a_idx_1;
        }
      }

      dynsim_2link_planar_B.R_c[0] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 2;
      dynsim_2link_planar_B.R_c[3] = -vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.i_n = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 1;
      dynsim_2link_planar_B.R_c[6] = vB->data[dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.R_c[1] = vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R_c[4] = 0.0;
      dynsim_2link_planar_B.R_c[7] = -vB->data[6 *
        dynsim_2link_planar_B.unnamed_idx_1];
      dynsim_2link_planar_B.R_c[2] = -vB->data[dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.R_c[5] = vB->data[6 *
        dynsim_2link_planar_B.unnamed_idx_1];
      dynsim_2link_planar_B.R_c[8] = 0.0;
      dynsim_2link_planar_B.R[3] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 5;
      dynsim_2link_planar_B.R[9] = -vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.i_n = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 4;
      dynsim_2link_planar_B.R[15] = vB->data[dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.R[4] = vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R[10] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 3;
      dynsim_2link_planar_B.R[16] = -vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R[5] = -vB->data[dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.R[11] = vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R[17] = 0.0;
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
           dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_c[3 *
          dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.b_k = 6 * (dynsim_2link_planar_B.i_n + 3);
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k] = 0.0;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 3] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_c[3 *
          dynsim_2link_planar_B.i_n + 1];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 1] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 1] = 0.0;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 4] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_c[3 *
          dynsim_2link_planar_B.i_n + 2];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 2] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 2] = 0.0;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 5] =
          dynsim_2link_planar_B.a_idx_1;
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += aB->data[(dynsim_2link_planar_B.m - 1)
            * 6 + dynsim_2link_planar_B.b_k] * X->
            data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_n];
        }

        dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.i_n] =
          dynsim_2link_planar_B.a_idx_1 +
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n];
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n] = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_n] * vJ->data[6 *
            dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.b_k] +
            dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n];
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n] =
            dynsim_2link_planar_B.a_idx_1;
        }

        aB->data[dynsim_2link_planar_B.i_n + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] =
          dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.i_n] +
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n];
      }

      dynsim_2link_planar_B.R_c[0] = 0.0;
      dynsim_2link_planar_B.R_c[3] = -dynsim_2link_planar_B.T[14];
      dynsim_2link_planar_B.R_c[6] = dynsim_2link_planar_B.T[13];
      dynsim_2link_planar_B.R_c[1] = dynsim_2link_planar_B.T[14];
      dynsim_2link_planar_B.R_c[4] = 0.0;
      dynsim_2link_planar_B.R_c[7] = -dynsim_2link_planar_B.T[12];
      dynsim_2link_planar_B.R_c[2] = -dynsim_2link_planar_B.T[13];
      dynsim_2link_planar_B.R_c[5] = dynsim_2link_planar_B.T[12];
      dynsim_2link_planar_B.R_c[8] = 0.0;
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
           dynsim_2link_planar_B.i_n++) {
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 3;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_n + 3 *
            dynsim_2link_planar_B.b_k;
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] = 0.0;
          dynsim_2link_planar_B.p = dynsim_2link_planar_B.b_k << 2;
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.p] *
            dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.i_n];
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.p + 1] *
            dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.i_n + 3];
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.p + 2] *
            dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.i_n + 6];
          dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 6 *
            dynsim_2link_planar_B.i_n] = dynsim_2link_planar_B.T
            [(dynsim_2link_planar_B.i_n << 2) + dynsim_2link_planar_B.b_k];
          dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 6 *
            (dynsim_2link_planar_B.i_n + 3)] = 0.0;
        }
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
           dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 3] =
          dynsim_2link_planar_B.R_b[3 * dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_n << 2;
        dynsim_2link_planar_B.q_size_tmp = 6 * (dynsim_2link_planar_B.i_n + 3);
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.q_size_tmp + 3] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.b_k];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 4] =
          dynsim_2link_planar_B.R_b[3 * dynsim_2link_planar_B.i_n + 1];
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.q_size_tmp + 4] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.b_k + 1];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 5] =
          dynsim_2link_planar_B.R_b[3 * dynsim_2link_planar_B.i_n + 2];
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.q_size_tmp + 5] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.b_k + 2];
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.p = dynsim_2link_planar_B.i_n + 6 *
            dynsim_2link_planar_B.b_k;
          dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.p] = 0.0;
          for (dynsim_2link_planar_B.q_size_tmp = 0;
               dynsim_2link_planar_B.q_size_tmp < 6;
               dynsim_2link_planar_B.q_size_tmp++) {
            dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.p] += Xtree->data[
              static_cast<int32_T>(dynsim_2link_planar_B.a_idx_0) - 1].f1[6 *
              dynsim_2link_planar_B.q_size_tmp + dynsim_2link_planar_B.i_n] *
              dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.b_k +
              dynsim_2link_planar_B.q_size_tmp];
          }
        }
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 36;
           dynsim_2link_planar_B.i_n++) {
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.i_n] =
          dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.i_n];
      }
    } else {
      dynsim_2link_planar_B.inner = S->size[1] - 1;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.i_n = 6 * dynsim_2link_planar_B.unnamed_idx_1 +
          dynsim_2link_planar_B.b_k;
        vB->data[dynsim_2link_planar_B.i_n] = vJ->data[dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.b_k] = 0.0;
      }

      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
           dynsim_2link_planar_B.inner; dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.b_k * 6 - 1;
        for (dynsim_2link_planar_B.q_size_tmp = 0;
             dynsim_2link_planar_B.q_size_tmp < 6;
             dynsim_2link_planar_B.q_size_tmp++) {
          dynsim_2link_planar_B.a_idx_1 = S->data[(dynsim_2link_planar_B.aoffset
            + dynsim_2link_planar_B.q_size_tmp) + 1] * 0.0 +
            dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp];
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp] =
            dynsim_2link_planar_B.a_idx_1;
        }
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += X->
            data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_n] *
            dynsim_2link_planar_B.a0[dynsim_2link_planar_B.b_k];
        }

        aB->data[dynsim_2link_planar_B.i_n + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] = dynsim_2link_planar_B.a_idx_1 +
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n];
      }

      dynsim_2link_planar_B.R_c[0] = 0.0;
      dynsim_2link_planar_B.R_c[3] = -dynsim_2link_planar_B.T[14];
      dynsim_2link_planar_B.R_c[6] = dynsim_2link_planar_B.T[13];
      dynsim_2link_planar_B.R_c[1] = dynsim_2link_planar_B.T[14];
      dynsim_2link_planar_B.R_c[4] = 0.0;
      dynsim_2link_planar_B.R_c[7] = -dynsim_2link_planar_B.T[12];
      dynsim_2link_planar_B.R_c[2] = -dynsim_2link_planar_B.T[13];
      dynsim_2link_planar_B.R_c[5] = dynsim_2link_planar_B.T[12];
      dynsim_2link_planar_B.R_c[8] = 0.0;
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
           dynsim_2link_planar_B.i_n++) {
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 3;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_n + 3 *
            dynsim_2link_planar_B.b_k;
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] = 0.0;
          dynsim_2link_planar_B.p = dynsim_2link_planar_B.b_k << 2;
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.p] *
            dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.i_n];
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.p + 1] *
            dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.i_n + 3];
          dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.p + 2] *
            dynsim_2link_planar_B.R_c[dynsim_2link_planar_B.i_n + 6];
          Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
            .f1[dynsim_2link_planar_B.b_k + 6 * dynsim_2link_planar_B.i_n] =
            dynsim_2link_planar_B.T[(dynsim_2link_planar_B.i_n << 2) +
            dynsim_2link_planar_B.b_k];
          Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
            .f1[dynsim_2link_planar_B.b_k + 6 * (dynsim_2link_planar_B.i_n + 3)]
            = 0.0;
        }
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
           dynsim_2link_planar_B.i_n++) {
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_n + 3] = dynsim_2link_planar_B.R_b[3 *
          dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_n << 2;
        dynsim_2link_planar_B.q_size_tmp = 6 * (dynsim_2link_planar_B.i_n + 3);
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.q_size_tmp + 3] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.b_k];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_n + 4] = dynsim_2link_planar_B.R_b[3 *
          dynsim_2link_planar_B.i_n + 1];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.q_size_tmp + 4] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.b_k + 1];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_n + 5] = dynsim_2link_planar_B.R_b[3 *
          dynsim_2link_planar_B.i_n + 2];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.q_size_tmp + 5] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.b_k + 2];
      }
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 36;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.i_n] = robot->
        Bodies[dynsim_2link_planar_B.unnamed_idx_1]->
        SpatialInertia[dynsim_2link_planar_B.i_n];
    }

    dynsim_2link_planar_B.R_c[0] = 0.0;
    dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 2;
    dynsim_2link_planar_B.R_c[3] = -vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.i_n = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 1;
    dynsim_2link_planar_B.R_c[6] = vB->data[dynsim_2link_planar_B.i_n];
    dynsim_2link_planar_B.R_c[1] = vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R_c[4] = 0.0;
    dynsim_2link_planar_B.R_c[7] = -vB->data[6 *
      dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.R_c[2] = -vB->data[dynsim_2link_planar_B.i_n];
    dynsim_2link_planar_B.R_c[5] = vB->data[6 *
      dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.R_c[8] = 0.0;
    dynsim_2link_planar_B.R[18] = 0.0;
    dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 5;
    dynsim_2link_planar_B.R[24] = -vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.i_n = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 4;
    dynsim_2link_planar_B.R[30] = vB->data[dynsim_2link_planar_B.i_n];
    dynsim_2link_planar_B.R[19] = vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R[25] = 0.0;
    dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 3;
    dynsim_2link_planar_B.R[31] = -vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R[20] = -vB->data[dynsim_2link_planar_B.i_n];
    dynsim_2link_planar_B.R[26] = vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R[32] = 0.0;
    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 3;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_c[3 *
        dynsim_2link_planar_B.i_n];
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 3] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * (dynsim_2link_planar_B.i_n + 3);
      dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 3] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_c[3 *
        dynsim_2link_planar_B.i_n + 1];
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 1] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 4] = 0.0;
      dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 4] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_c[3 *
        dynsim_2link_planar_B.i_n + 2];
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 2] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_n + 5] = 0.0;
      dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 5] =
        dynsim_2link_planar_B.a_idx_1;
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.i_n] = 0.0;
      dynsim_2link_planar_B.b_I_n[dynsim_2link_planar_B.i_n] = 0.0;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.a_idx_0 = dynsim_2link_planar_B.b_I[6 *
          dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.q_size_tmp = 6 *
          dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.b_k;
        dynsim_2link_planar_B.a_idx_1 = vB->
          data[dynsim_2link_planar_B.q_size_tmp] * dynsim_2link_planar_B.a_idx_0
          + dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.a_idx_0 = aB->
          data[dynsim_2link_planar_B.q_size_tmp] * dynsim_2link_planar_B.a_idx_0
          + dynsim_2link_planar_B.b_I_n[dynsim_2link_planar_B.i_n];
        dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.i_n] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.b_I_n[dynsim_2link_planar_B.i_n] =
          dynsim_2link_planar_B.a_idx_0;
      }
    }

    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n] = 0.0;
      dynsim_2link_planar_B.a_idx_1 = 0.0;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.a_idx_1 += Xtree->
          data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_n + dynsim_2link_planar_B.b_k] * fext[6 *
          dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.b_k];
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n] +=
          dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.b_k +
          dynsim_2link_planar_B.i_n] *
          dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.b_k];
      }

      f->data[dynsim_2link_planar_B.i_n + 6 *
        dynsim_2link_planar_B.unnamed_idx_1] =
        (dynsim_2link_planar_B.b_I_n[dynsim_2link_planar_B.i_n] +
         dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_n]) -
        dynsim_2link_planar_B.a_idx_1;
    }
  }

  dynsim_2link_pla_emxFree_real_T(&aB);
  dynsim_2link_pla_emxFree_real_T(&vB);
  dynsim_2link_pla_emxFree_real_T(&vJ);
  dynsim_2lin_emxFree_f_cell_wrap(&Xtree);
  dynsim_2link_planar_B.q_size_tmp = static_cast<int32_T>(((-1.0 -
    dynsim_2link_planar_B.nb) + 1.0) / -1.0) - 1;
  dynsim_2link_pla_emxInit_real_T(&taui, 1);
  dynsim_2link_pla_emxInit_char_T(&a, 2);
  if (0 <= dynsim_2link_planar_B.q_size_tmp) {
    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 5;
         dynsim_2link_planar_B.i_n++) {
      dynsim_2link_planar_B.b_jz[dynsim_2link_planar_B.i_n] =
        tmp[dynsim_2link_planar_B.i_n];
    }
  }

  for (dynsim_2link_planar_B.loop_ub_tmp = 0; dynsim_2link_planar_B.loop_ub_tmp <=
       dynsim_2link_planar_B.q_size_tmp; dynsim_2link_planar_B.loop_ub_tmp++) {
    dynsim_2link_planar_B.a_idx_0 = dynsim_2link_planar_B.nb +
      -static_cast<real_T>(dynsim_2link_planar_B.loop_ub_tmp);
    dynsim_2link_planar_B.p = static_cast<int32_T>(dynsim_2link_planar_B.a_idx_0);
    dynsim_2link_planar_B.inner = dynsim_2link_planar_B.p - 1;
    obj = robot->Bodies[dynsim_2link_planar_B.inner];
    dynsim_2link_planar_B.i_n = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(a, dynsim_2link_planar_B.i_n);
    dynsim_2link_planar_B.b_k = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <=
         dynsim_2link_planar_B.b_k; dynsim_2link_planar_B.i_n++) {
      a->data[dynsim_2link_planar_B.i_n] = obj->JointInternal.Type->
        data[dynsim_2link_planar_B.i_n];
    }

    dynsim_2link_planar_B.b_bool = false;
    if (a->size[1] == 5) {
      dynsim_2link_planar_B.i_n = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.i_n - 1 < 5) {
          dynsim_2link_planar_B.unnamed_idx_1 = dynsim_2link_planar_B.i_n - 1;
          if (a->data[dynsim_2link_planar_B.unnamed_idx_1] !=
              dynsim_2link_planar_B.b_jz[dynsim_2link_planar_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.i_n++;
          }
        } else {
          dynsim_2link_planar_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!dynsim_2link_planar_B.b_bool) {
      obj = robot->Bodies[dynsim_2link_planar_B.inner];
      dynsim_2link_planar_B.i_n = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(S, dynsim_2link_planar_B.i_n);
      dynsim_2link_planar_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <=
           dynsim_2link_planar_B.b_k; dynsim_2link_planar_B.i_n++) {
        S->data[dynsim_2link_planar_B.i_n] = obj->
          JointInternal.MotionSubspace->data[dynsim_2link_planar_B.i_n];
      }

      dynsim_2link_planar_B.m = S->size[1] - 1;
      dynsim_2link_planar_B.i_n = taui->size[0];
      taui->size[0] = S->size[1];
      dynsim_emxEnsureCapacity_real_T(taui, dynsim_2link_planar_B.i_n);
      for (dynsim_2link_planar_B.unnamed_idx_1 = 0;
           dynsim_2link_planar_B.unnamed_idx_1 <= dynsim_2link_planar_B.m;
           dynsim_2link_planar_B.unnamed_idx_1++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.unnamed_idx_1 * 6
          - 1;
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += f->data[(static_cast<int32_T>
            (dynsim_2link_planar_B.a_idx_0) - 1) * 6 + dynsim_2link_planar_B.b_k]
            * S->data[(dynsim_2link_planar_B.aoffset + dynsim_2link_planar_B.b_k)
            + 1];
        }

        taui->data[dynsim_2link_planar_B.unnamed_idx_1] =
          dynsim_2link_planar_B.a_idx_1;
      }

      dynsim_2link_planar_B.b_idx_0 = robot->
        VelocityDoFMap[dynsim_2link_planar_B.p - 1];
      dynsim_2link_planar_B.b_idx_1 = robot->
        VelocityDoFMap[dynsim_2link_planar_B.p + 5];
      if (dynsim_2link_planar_B.b_idx_0 > dynsim_2link_planar_B.b_idx_1) {
        dynsim_2link_planar_B.b_k = 0;
        dynsim_2link_planar_B.i_n = 0;
      } else {
        dynsim_2link_planar_B.b_k = static_cast<int32_T>
          (dynsim_2link_planar_B.b_idx_0) - 1;
        dynsim_2link_planar_B.i_n = static_cast<int32_T>
          (dynsim_2link_planar_B.b_idx_1);
      }

      dynsim_2link_planar_B.unnamed_idx_1 = dynsim_2link_planar_B.i_n -
        dynsim_2link_planar_B.b_k;
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n <
           dynsim_2link_planar_B.unnamed_idx_1; dynsim_2link_planar_B.i_n++) {
        tau[dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_n] = taui->
          data[dynsim_2link_planar_B.i_n];
      }
    }

    dynsim_2link_planar_B.a_idx_0 = robot->Bodies[dynsim_2link_planar_B.inner]
      ->ParentIndex;
    if (dynsim_2link_planar_B.a_idx_0 > 0.0) {
      dynsim_2link_planar_B.m = static_cast<int32_T>
        (dynsim_2link_planar_B.a_idx_0);
      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += f->data[(dynsim_2link_planar_B.p - 1)
            * 6 + dynsim_2link_planar_B.b_k] * X->
            data[dynsim_2link_planar_B.inner].f1[6 * dynsim_2link_planar_B.i_n +
            dynsim_2link_planar_B.b_k];
        }

        dynsim_2link_planar_B.a0[dynsim_2link_planar_B.i_n] = f->data
          [(dynsim_2link_planar_B.m - 1) * 6 + dynsim_2link_planar_B.i_n] +
          dynsim_2link_planar_B.a_idx_1;
      }

      for (dynsim_2link_planar_B.i_n = 0; dynsim_2link_planar_B.i_n < 6;
           dynsim_2link_planar_B.i_n++) {
        f->data[dynsim_2link_planar_B.i_n + 6 * (dynsim_2link_planar_B.m - 1)] =
          dynsim_2link_planar_B.a0[dynsim_2link_planar_B.i_n];
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&a);
  dynsim_2link_pla_emxFree_real_T(&taui);
  dynsim_2link_pla_emxFree_real_T(&S);
  dynsim_2link_pla_emxFree_real_T(&f);
  dynsim_2lin_emxFree_f_cell_wrap(&X);
}

static void matlabCodegenHa_awtnueqcp3y53m2(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2link_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->Type);
  dynsim_2link_pla_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_m_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_n_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void matlabCodegenHandl_awtnueqcp3y5(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2link_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->Type, 2);
  dynsim_2link_pla_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_m_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_n_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static l_robotics_manip_internal_Rig_T *dynsim_2lin_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.01963977352686529, 0.00029712556424630341,
    -2.0555635461349874E-6, 0.0, 0.000159441301187506, 0.023879428445548689,
    0.00029712556424630341, 7.2134270934731868E-5, 0.00014018503386442084,
    -0.000159441301187506, 0.0, 0.0003592140353393829, -2.0555635461349874E-6,
    0.00014018503386442084, 0.01964319797436196, -0.023879428445548689,
    -0.0003592140353393829, 0.0, 0.0, -0.000159441301187506,
    -0.023879428445548689, 0.0785942338762368, 0.0, 0.0, 0.000159441301187506,
    0.0, -0.0003592140353393829, 0.0, 0.0785942338762368, 0.0,
    0.023879428445548689, 0.0003592140353393829, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.99999999997301514, 7.3464102066435871E-6,
    -6.9389E-16, 0.0, 2.6984177572320606E-11, 3.6732051032474579E-6,
    0.99999999999325373, 0.0, 7.3464102065940289E-6, 0.99999999996626887,
    -3.6732051033465739E-6, 0.0, 0.057188, 0.0059831, 0.13343, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 0.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim_2l_RigidBody_RigidBody_a
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.013211023632997332, 0.0010249919084558737,
    -0.0012646868357119422, 0.0, 0.0068913182657821814, 0.0056052022217139017,
    0.0010249919084558737, 0.0082026017376173183, 0.0064295305818937417,
    -0.0068913182657821814, 0.0, 0.0013235778724120266, -0.0012646868357119422,
    0.0064295305818937417, 0.0054794684392445022, -0.0056052022217139017,
    -0.0013235778724120266, 0.0, 0.0, -0.0068913182657821814,
    -0.0056052022217139017, 0.0785942338762368, 0.0, 0.0, 0.0068913182657821814,
    0.0, -0.0013235778724120266, 0.0, 0.0785942338762368, 0.0,
    0.0056052022217139017, 0.0013235778724120266, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 0.88847119465357549, -0.10400474353252914,
    0.44699211356978236, 0.0, -0.29079372810919513, 0.62592701137950757,
    0.7236396783744472, 0.0, -0.35504639691623957, -0.77291551268444,
    0.52587419245342837, 0.0, 0.0, 0.18967, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { -0.88847134048214615, 0.29080043874549294,
    0.3550405356678123, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 1.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -0.88847134048214615;
  obj->JointInternal.JointAxisInternal[1] = 0.29080043874549294;
  obj->JointInternal.JointAxisInternal[2] = 0.3550405356678123;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim_2_RigidBody_RigidBody_aw
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.013113488639879966, 0.00015340966821489427,
    -9.3644949998630182E-8, 0.0, -0.00031079476582565222, 0.0075648906322210207,
    0.00015340966821489427, 7.3039831742822541E-5, -0.00012165646429114304,
    0.00031079476582565222, 0.0, -0.00035921403533938209, -9.3644949998630182E-8,
    -0.00012165646429114304, 0.013116007526568545, -0.0075648906322210207,
    0.00035921403533938209, 0.0, 0.0, 0.00031079476582565222,
    -0.0075648906322210207, 0.0785942338762368, 0.0, 0.0,
    -0.00031079476582565222, 0.0, 0.00035921403533938209, 0.0,
    0.0785942338762368, 0.0, 0.0075648906322210207, -0.00035921403533938209, 0.0,
    0.0, 0.0, 0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.88847685238438157, 0.29078066152542581,
    0.35504294058603364, 0.0, 0.10401076381482063, -0.62592577039760033,
    0.77291570754049777, 0.0, 0.44697946685256096, 0.72364600243144184,
    0.5258762396012906, 0.0, -0.024558, 0.12737, -0.16578, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 2.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_awt
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.017498226510398153, -2.6413489892615174E-6,
    0.00027276976237199086, 0.0, -0.020046014166386395, -0.00031079476582562474,
    -2.6413489892615174E-6, 0.01750050956753612, 0.00017101209351135795,
    0.020046014166386395, 0.0, 0.000332415222770069, 0.00027276976237199086,
    0.00017101209351135795, 7.2804002192213048E-5, 0.00031079476582562474,
    -0.000332415222770069, 0.0, 0.0, 0.020046014166386395,
    0.00031079476582562474, 0.0785942338762368, 0.0, 0.0, -0.020046014166386395,
    0.0, -0.000332415222770069, 0.0, 0.0785942338762368, 0.0,
    -0.00031079476582562474, 0.000332415222770069, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0088, -0.1588, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 3.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsim_RigidBody_RigidBody_awtn
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.022591118157548329, 2.6413489892594934E-6,
    0.00030777156611164522, 0.0, 0.028321627798935184, -0.00031079476582561606,
    2.6413489892594934E-6, 0.022593401214686296, -0.00020373736194570198,
    -0.028321627798935184, 0.0, -0.00033241522276980414, 0.00030777156611164522,
    -0.00020373736194570198, 7.2804002192206746E-5, 0.00031079476582561606,
    0.00033241522276980414, 0.0, 0.0, -0.028321627798935184,
    0.00031079476582561606, 0.0785942338762368, 0.0, 0.0, 0.028321627798935184,
    0.0, 0.00033241522276980414, 0.0, 0.0785942338762368, 0.0,
    -0.00031079476582561606, -0.00033241522276980414, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.99999999997301514, -7.3464102066435871E-6,
    -1.1102E-14, 0.0, -7.3464102064452637E-6, 0.99999999994603028,
    -7.3464102066435871E-6, 0.0, 5.3980844924276475E-11, -7.3464102064452637E-6,
    -0.99999999997301514, 0.0, 0.0, 0.0, -0.1053, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 4.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *dynsi_RigidBody_RigidBody_awtnu
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 1.0065981357838704E-5, 3.0729164240456609E-9,
    5.3277371841056389E-9, 0.0, -1.7778570846549331E-7, -0.00026077765450549392,
    3.0729164240456609E-9, 1.4574495784752631E-5, 1.7665241858879138E-9,
    1.7778570846549331E-7, 0.0, -3.0929847108333021E-7, 5.3277371841056389E-9,
    1.7665241858879138E-9, 1.006005411704865E-5, 0.00026077765450549392,
    3.0929847108333021E-7, 0.0, 0.0, 1.7778570846549331E-7,
    0.00026077765450549392, 0.0279702497322662, 0.0, 0.0, -1.7778570846549331E-7,
    0.0, 3.0929847108333021E-7, 0.0, 0.0279702497322662, 0.0,
    -0.00026077765450549392, -3.0929847108333021E-7, 0.0, 0.0, 0.0,
    0.0279702497322662 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -1.249E-14, 0.0,
    -1.2489999999915739E-14, -3.6732051033465739E-6, -0.99999999999325373, 0.0,
    -4.587833174079871E-20, 0.99999999999325373, -3.6732051033465739E-6, 0.0,
    -0.0039, 0.0, 0.1636, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 5.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static m_robotics_manip_internal_Rig_T *dyns_RigidBody_RigidBody_awtnue
  (m_robotics_manip_internal_Rig_T *obj)
{
  m_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      dynsim_2link_planar_B.b_d[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] !=
              dynsim_2link_planar_B.b_d[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_2link_planar_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_2link_planar_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynsim_2link_planar_B.msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      dynsim_2link_planar_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (n_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Rig_T *iobj_0,
   l_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Rig_T
   *iobj_2, l_robotics_manip_internal_Rig_T *iobj_3,
   l_robotics_manip_internal_Rig_T *iobj_4, l_robotics_manip_internal_Rig_T
   *iobj_5)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int32_T i;
  static const int8_T tmp[12] = { 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6 };

  b_obj = obj;
  obj->Bodies[0] = dynsim_2lin_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = dynsim_2l_RigidBody_RigidBody_a(iobj_5);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = dynsim_2_RigidBody_RigidBody_aw(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = dynsim__RigidBody_RigidBody_awt(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = dynsim_RigidBody_RigidBody_awtn(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = dynsi_RigidBody_RigidBody_awtnu(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->NumBodies = 6.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 12; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 12; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  dyns_RigidBody_RigidBody_awtnue(&obj->Base);
  return b_obj;
}

// Model step function
void dynsim_2link_planar_step(void)
{
  robotics_slmanip_internal_blo_T *obj;
  emxArray_real_T_dynsim_2link__T *L;
  emxArray_real_T_dynsim_2link__T *lambda;
  emxArray_real_T_dynsim_2link__T *H;
  emxArray_real_T_dynsim_2link__T *tmp;
  static const char_T h[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '1' };

  static const char_T i[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '2' };

  static const char_T j[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '3' };

  static const char_T k[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '4' };

  static const char_T l[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '5' };

  static const char_T m[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '6' };

  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&dynsim_2link_planar_M->solverInfo,
                          ((dynsim_2link_planar_M->Timing.clockTick0+1)*
      dynsim_2link_planar_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(dynsim_2link_planar_M)) {
    dynsim_2link_planar_M->Timing.t[0] = rtsiGetT
      (&dynsim_2link_planar_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S11>/SourceBlock' incorporates:
    //   Inport: '<S14>/In1'

    dynsim_2link_pl_SystemCore_step(&dynsim_2link_planar_B.b_varargout_1,
      dynsim_2link_planar_B.b_varargout_2_Data,
      &dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Curr,
      &dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Rece,
      &dynsim_2link_planar_B.b_varargout_2_Layout_DataOffset,
      dynsim_2link_planar_B.b_varargout_2_Layout_Dim,
      &dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_Inf,
      &dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_I_f);

    // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S14>/Enable'

    if (dynsim_2link_planar_B.b_varargout_1) {
      memcpy(&dynsim_2link_planar_B.In1.Data[0],
             &dynsim_2link_planar_B.b_varargout_2_Data[0], sizeof(real_T) << 7U);
      dynsim_2link_planar_B.In1.Data_SL_Info.CurrentLength =
        dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Curr;
      dynsim_2link_planar_B.In1.Data_SL_Info.ReceivedLength =
        dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Rece;
      dynsim_2link_planar_B.In1.Layout.DataOffset =
        dynsim_2link_planar_B.b_varargout_2_Layout_DataOffset;
      memcpy(&dynsim_2link_planar_B.In1.Layout.Dim[0],
             &dynsim_2link_planar_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension) << 4U);
      dynsim_2link_planar_B.In1.Layout.Dim_SL_Info.CurrentLength =
        dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_Inf;
      dynsim_2link_planar_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_I_f;
    }

    // End of MATLABSystem: '<S11>/SourceBlock'
    // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  // MATLABSystem: '<S13>/Get Parameter'
  ParamGet_dynsim_2link_planar_112.get_parameter(&dynsim_2link_planar_B.vNum);

  // MATLABSystem: '<S13>/Get Parameter1'
  ParamGet_dynsim_2link_planar_113.get_parameter(&dynsim_2link_planar_B.k);

  // MATLABSystem: '<S13>/Get Parameter4'
  ParamGet_dynsim_2link_planar_129.get_parameter(&dynsim_2link_planar_B.j);

  // MATLABSystem: '<S13>/Get Parameter6'
  ParamGet_dynsim_2link_planar_131.get_parameter(&dynsim_2link_planar_B.value);

  // MATLABSystem: '<S13>/Get Parameter7'
  ParamGet_dynsim_2link_planar_132.get_parameter(&dynsim_2link_planar_B.value_d);

  // MATLABSystem: '<S13>/Get Parameter8'
  ParamGet_dynsim_2link_planar_133.get_parameter(&dynsim_2link_planar_B.value_l);

  // Integrator: '<S1>/Position' incorporates:
  //   MATLABSystem: '<S13>/Get Parameter'
  //   MATLABSystem: '<S13>/Get Parameter1'
  //   MATLABSystem: '<S13>/Get Parameter4'
  //   MATLABSystem: '<S13>/Get Parameter6'
  //   MATLABSystem: '<S13>/Get Parameter7'
  //   MATLABSystem: '<S13>/Get Parameter8'

  if (dynsim_2link_planar_DW.Position_IWORK != 0) {
    dynsim_2link_planar_X.Position_CSTATE[0] = dynsim_2link_planar_B.vNum;
    dynsim_2link_planar_X.Position_CSTATE[1] = dynsim_2link_planar_B.k;
    dynsim_2link_planar_X.Position_CSTATE[2] = dynsim_2link_planar_B.j;
    dynsim_2link_planar_X.Position_CSTATE[3] = dynsim_2link_planar_B.value;
    dynsim_2link_planar_X.Position_CSTATE[4] = dynsim_2link_planar_B.value_d;
    dynsim_2link_planar_X.Position_CSTATE[5] = dynsim_2link_planar_B.value_l;
  }

  // MATLABSystem: '<S13>/Get Parameter2'
  ParamGet_dynsim_2link_planar_117.get_parameter(&dynsim_2link_planar_B.vNum);

  // MATLABSystem: '<S13>/Get Parameter3'
  ParamGet_dynsim_2link_planar_118.get_parameter(&dynsim_2link_planar_B.k);

  // MATLABSystem: '<S13>/Get Parameter5'
  ParamGet_dynsim_2link_planar_130.get_parameter(&dynsim_2link_planar_B.j);

  // MATLABSystem: '<S13>/Get Parameter9'
  ParamGet_dynsim_2link_planar_134.get_parameter(&dynsim_2link_planar_B.value);

  // MATLABSystem: '<S13>/Get Parameter10'
  ParamGet_dynsim_2link_planar_135.get_parameter(&dynsim_2link_planar_B.value_d);

  // MATLABSystem: '<S13>/Get Parameter11'
  ParamGet_dynsim_2link_planar_136.get_parameter(&dynsim_2link_planar_B.value_l);

  // Integrator: '<S1>/Velocity' incorporates:
  //   MATLABSystem: '<S13>/Get Parameter10'
  //   MATLABSystem: '<S13>/Get Parameter11'
  //   MATLABSystem: '<S13>/Get Parameter2'
  //   MATLABSystem: '<S13>/Get Parameter3'
  //   MATLABSystem: '<S13>/Get Parameter5'
  //   MATLABSystem: '<S13>/Get Parameter9'

  if (dynsim_2link_planar_DW.Velocity_IWORK != 0) {
    dynsim_2link_planar_X.Velocity_CSTATE[0] = dynsim_2link_planar_B.vNum;
    dynsim_2link_planar_X.Velocity_CSTATE[1] = dynsim_2link_planar_B.k;
    dynsim_2link_planar_X.Velocity_CSTATE[2] = dynsim_2link_planar_B.j;
    dynsim_2link_planar_X.Velocity_CSTATE[3] = dynsim_2link_planar_B.value;
    dynsim_2link_planar_X.Velocity_CSTATE[4] = dynsim_2link_planar_B.value_d;
    dynsim_2link_planar_X.Velocity_CSTATE[5] = dynsim_2link_planar_B.value_l;
  }

  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i < 6;
       dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.Velocity[dynsim_2link_planar_B.i] =
      dynsim_2link_planar_X.Velocity_CSTATE[dynsim_2link_planar_B.i];
  }

  // End of Integrator: '<S1>/Velocity'
  dynsim_2link_pla_emxInit_real_T(&L, 2);
  dynsim_2link_pla_emxInit_real_T(&lambda, 2);
  dynsim_2link_pla_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S12>/MATLAB System' incorporates:
  //   Constant: '<S1>/Constant'
  //   Integrator: '<S1>/Position'

  obj = &dynsim_2link_planar_DW.obj;
  RigidBodyTreeDynamics_massMatri(&dynsim_2link_planar_DW.obj.TreeInternal,
    dynsim_2link_planar_X.Position_CSTATE, L, lambda);
  dynsim_2link_planar_B.vNum = obj->TreeInternal.VelocityNumber;
  dynsim_2link_planar_B.vNum_idx_0_tmp = static_cast<int32_T>
    (dynsim_2link_planar_B.vNum);
  dynsim_2link_planar_B.j_j = tmp->size[0];
  tmp->size[0] = dynsim_2link_planar_B.vNum_idx_0_tmp;
  dynsim_emxEnsureCapacity_real_T(tmp, dynsim_2link_planar_B.j_j);
  for (dynsim_2link_planar_B.j_j = 0; dynsim_2link_planar_B.j_j <
       dynsim_2link_planar_B.vNum_idx_0_tmp; dynsim_2link_planar_B.j_j++) {
    tmp->data[dynsim_2link_planar_B.j_j] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj->TreeInternal,
    dynsim_2link_planar_X.Position_CSTATE, dynsim_2link_planar_B.Velocity,
    dynsim_2link_planar_P.Constant_Value_f, dynsim_2link_planar_B.MATLABSystem);
  dynsim_2link_pla_emxFree_real_T(&tmp);

  // MATLABSystem: '<S12>/MATLAB System'
  for (dynsim_2link_planar_B.j_j = 0; dynsim_2link_planar_B.j_j < 6;
       dynsim_2link_planar_B.j_j++) {
    dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.j_j] =
      dynsim_2link_planar_B.In1.Data[dynsim_2link_planar_B.j_j + 1] -
      dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.j_j];
  }

  if ((L->size[0] == 0) || (L->size[1] == 0)) {
    dynsim_2link_planar_B.iend = 0;
  } else {
    dynsim_2link_planar_B.i = L->size[0];
    dynsim_2link_planar_B.iend = L->size[1];
    if (dynsim_2link_planar_B.i > dynsim_2link_planar_B.iend) {
      dynsim_2link_planar_B.iend = dynsim_2link_planar_B.i;
    }
  }

  dynsim_2link_pla_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S12>/MATLAB System'
  dynsim_2link_planar_B.j_j = H->size[0] * H->size[1];
  H->size[0] = L->size[0];
  H->size[1] = L->size[1];
  dynsim_emxEnsureCapacity_real_T(H, dynsim_2link_planar_B.j_j);
  dynsim_2link_planar_B.i = L->size[0] * L->size[1] - 1;
  for (dynsim_2link_planar_B.j_j = 0; dynsim_2link_planar_B.j_j <=
       dynsim_2link_planar_B.i; dynsim_2link_planar_B.j_j++) {
    H->data[dynsim_2link_planar_B.j_j] = L->data[dynsim_2link_planar_B.j_j];
  }

  dynsim_2link_planar_B.n = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (dynsim_2link_planar_B.iend)) + 1.0) / -1.0) - 1;
  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
       dynsim_2link_planar_B.n; dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.j = static_cast<real_T>(dynsim_2link_planar_B.iend) +
      -static_cast<real_T>(dynsim_2link_planar_B.i);
    dynsim_2link_planar_B.j_j = static_cast<int32_T>(dynsim_2link_planar_B.j);
    dynsim_2link_planar_B.MATLABSystem_tmp = dynsim_2link_planar_B.j_j - 1;
    H->data[(static_cast<int32_T>(dynsim_2link_planar_B.j) + H->size[0] * (
              static_cast<int32_T>(dynsim_2link_planar_B.j) - 1)) - 1] = sqrt
      (H->data[(dynsim_2link_planar_B.MATLABSystem_tmp * H->size[0] +
                dynsim_2link_planar_B.j_j) - 1]);
    dynsim_2link_planar_B.k = lambda->
      data[dynsim_2link_planar_B.MATLABSystem_tmp];
    while (dynsim_2link_planar_B.k > 0.0) {
      dynsim_2link_planar_B.i_o = static_cast<int32_T>(dynsim_2link_planar_B.k)
        - 1;
      H->data[(static_cast<int32_T>(dynsim_2link_planar_B.j) + H->size[0] * (
                static_cast<int32_T>(dynsim_2link_planar_B.k) - 1)) - 1] =
        H->data[(dynsim_2link_planar_B.i_o * H->size[0] +
                 dynsim_2link_planar_B.j_j) - 1] / H->data[((static_cast<int32_T>
        (dynsim_2link_planar_B.j) - 1) * H->size[0] + static_cast<int32_T>
        (dynsim_2link_planar_B.j)) - 1];
      dynsim_2link_planar_B.k = lambda->data[dynsim_2link_planar_B.i_o];
    }

    dynsim_2link_planar_B.k = lambda->
      data[dynsim_2link_planar_B.MATLABSystem_tmp];
    while (dynsim_2link_planar_B.k > 0.0) {
      dynsim_2link_planar_B.j = dynsim_2link_planar_B.k;
      while (dynsim_2link_planar_B.j > 0.0) {
        dynsim_2link_planar_B.MATLABSystem_tmp = static_cast<int32_T>
          (dynsim_2link_planar_B.j) - 1;
        H->data[(static_cast<int32_T>(dynsim_2link_planar_B.k) + H->size[0] * (
                  static_cast<int32_T>(dynsim_2link_planar_B.j) - 1)) - 1] =
          H->data[(dynsim_2link_planar_B.MATLABSystem_tmp * H->size[0] +
                   static_cast<int32_T>(dynsim_2link_planar_B.k)) - 1] - H->
          data[((static_cast<int32_T>(dynsim_2link_planar_B.k) - 1) * H->size[0]
                + dynsim_2link_planar_B.j_j) - 1] * H->data
          [((static_cast<int32_T>(dynsim_2link_planar_B.j) - 1) * H->size[0] +
            dynsim_2link_planar_B.j_j) - 1];
        dynsim_2link_planar_B.j = lambda->
          data[dynsim_2link_planar_B.MATLABSystem_tmp];
      }

      dynsim_2link_planar_B.k = lambda->data[static_cast<int32_T>
        (dynsim_2link_planar_B.k) - 1];
    }
  }

  dynsim_2link_planar_B.j_j = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  dynsim_emxEnsureCapacity_real_T(L, dynsim_2link_planar_B.j_j);
  dynsim_2link_planar_B.i = H->size[0] * H->size[1] - 1;
  for (dynsim_2link_planar_B.j_j = 0; dynsim_2link_planar_B.j_j <=
       dynsim_2link_planar_B.i; dynsim_2link_planar_B.j_j++) {
    L->data[dynsim_2link_planar_B.j_j] = H->data[dynsim_2link_planar_B.j_j];
  }

  dynsim_2link_planar_B.n = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    dynsim_2link_planar_B.iend = 0;
    for (dynsim_2link_planar_B.j_j = 2; dynsim_2link_planar_B.j_j <=
         dynsim_2link_planar_B.n; dynsim_2link_planar_B.j_j++) {
      for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
           dynsim_2link_planar_B.iend; dynsim_2link_planar_B.i++) {
        L->data[dynsim_2link_planar_B.i + L->size[0] *
          (dynsim_2link_planar_B.j_j - 1)] = 0.0;
      }

      if (dynsim_2link_planar_B.iend + 1 < H->size[0]) {
        dynsim_2link_planar_B.iend++;
      }
    }
  }

  dynsim_2link_pla_emxFree_real_T(&H);

  // MATLABSystem: '<S12>/MATLAB System'
  dynsim_2link_planar_B.n = static_cast<int32_T>(((-1.0 -
    dynsim_2link_planar_B.vNum) + 1.0) / -1.0) - 1;
  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
       dynsim_2link_planar_B.n; dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.iend = static_cast<int32_T>(dynsim_2link_planar_B.vNum
      + -static_cast<real_T>(dynsim_2link_planar_B.i));
    dynsim_2link_planar_B.j_j = dynsim_2link_planar_B.iend - 1;
    dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.j_j] /= L->data
      [(dynsim_2link_planar_B.j_j * L->size[0] + dynsim_2link_planar_B.iend) - 1];
    dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.j_j];
    while (dynsim_2link_planar_B.j > 0.0) {
      dynsim_2link_planar_B.MATLABSystem_tmp = static_cast<int32_T>
        (dynsim_2link_planar_B.j) - 1;
      dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.MATLABSystem_tmp]
        -= L->data[(dynsim_2link_planar_B.MATLABSystem_tmp * L->size[0] +
                    dynsim_2link_planar_B.iend) - 1] *
        dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.j_j];
      dynsim_2link_planar_B.j = lambda->
        data[dynsim_2link_planar_B.MATLABSystem_tmp];
    }
  }

  dynsim_2link_planar_B.vNum_idx_0_tmp--;
  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
       dynsim_2link_planar_B.vNum_idx_0_tmp; dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.i];
    while (dynsim_2link_planar_B.j > 0.0) {
      dynsim_2link_planar_B.j_j = static_cast<int32_T>(dynsim_2link_planar_B.j)
        - 1;
      dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.i] -= L->
        data[dynsim_2link_planar_B.j_j * L->size[0] + dynsim_2link_planar_B.i] *
        dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.j_j];
      dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.j_j];
    }

    dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.i] /= L->data
      [L->size[0] * dynsim_2link_planar_B.i + dynsim_2link_planar_B.i];
  }

  dynsim_2link_pla_emxFree_real_T(&lambda);
  dynsim_2link_pla_emxFree_real_T(&L);

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  dynsim_2link_planar_B.vNum = dynsim_2link_planar_M->Timing.t[0];

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Clock: '<Root>/Clock1'
  //   Constant: '<S5>/Constant'

  dynsim_2link_planar_B.msg_g = dynsim_2link_planar_P.Constant_Value;
  if (dynsim_2link_planar_B.vNum < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.k = ceil(dynsim_2link_planar_B.vNum);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.k = floor(dynsim_2link_planar_B.vNum);
  }

  dynsim_2link_planar_B.msg_g.Header.Stamp.Sec = dynsim_2link_planar_B.k;
  dynsim_2link_planar_B.j = (dynsim_2link_planar_B.vNum -
    dynsim_2link_planar_B.k) * 1.0E+9;
  if (dynsim_2link_planar_B.j < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.j = ceil(dynsim_2link_planar_B.j);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.j = floor(dynsim_2link_planar_B.j);
  }

  dynsim_2link_planar_B.msg_g.Header.Stamp.Nsec = dynsim_2link_planar_B.j;
  dynsim_2link_planar_B.msg_g.Name_SL_Info.CurrentLength = 3U;
  dynsim_2link_planar_B.msg_g.Position_SL_Info.CurrentLength = 3U;
  dynsim_2link_planar_B.msg_g.Velocity_SL_Info.CurrentLength = 3U;
  dynsim_2link_planar_B.msg_g.Name[0].Data_SL_Info.CurrentLength = 11U;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Integrator: '<S1>/Position'

  dynsim_2link_planar_B.value = cos(dynsim_2link_planar_X.Position_CSTATE[0]);
  dynsim_2link_planar_B.value_d = cos(dynsim_2link_planar_X.Position_CSTATE[1]);
  dynsim_2link_planar_B.value_l = sin(dynsim_2link_planar_X.Position_CSTATE[0]);
  dynsim_2link_planar_B.d2 = sin(dynsim_2link_planar_X.Position_CSTATE[1]);
  dynsim_2link_planar_B.d4 = sin(dynsim_2link_planar_X.Position_CSTATE[2]);
  dynsim_2link_planar_B.d_o = cos(dynsim_2link_planar_X.Position_CSTATE[3]);
  dynsim_2link_planar_B.d6 = cos(dynsim_2link_planar_X.Position_CSTATE[2]);
  dynsim_2link_planar_B.d1 = sin(dynsim_2link_planar_X.Position_CSTATE[3]);
  dynsim_2link_planar_B.d3 = 4.967757600021511E+15 *
    dynsim_2link_planar_B.value_l;
  dynsim_2link_planar_B.d5 = dynsim_2link_planar_B.value *
    dynsim_2link_planar_B.value_d - dynsim_2link_planar_B.d3 *
    dynsim_2link_planar_B.d2 / 8.1129638414606682E+31;
  dynsim_2link_planar_B.d17 = 4.967757600021511E+15 *
    dynsim_2link_planar_B.value_d;
  dynsim_2link_planar_B.d7 = dynsim_2link_planar_B.d17 *
    dynsim_2link_planar_B.value_l / 8.1129638414606682E+31 +
    dynsim_2link_planar_B.value * dynsim_2link_planar_B.d2;
  dynsim_2link_planar_B.d10 = 4.967757600021511E+15 * dynsim_2link_planar_B.d4;
  dynsim_2link_planar_B.d11 = 4.967757600021511E+15 * dynsim_2link_planar_B.d6;
  dynsim_2link_planar_B.d15 = dynsim_2link_planar_B.d5 *
    dynsim_2link_planar_B.d10 / 8.1129638414606682E+31;
  dynsim_2link_planar_B.d16 = dynsim_2link_planar_B.d7 *
    dynsim_2link_planar_B.d11 / 8.1129638414606682E+31;
  dynsim_2link_planar_B.d8 = (dynsim_2link_planar_B.d15 -
    dynsim_2link_planar_B.value_l) + dynsim_2link_planar_B.d16;
  dynsim_2link_planar_B.d9 = (4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[0]) / 8.1129638414606682E+31 + cos
    (dynsim_2link_planar_X.Position_CSTATE[0]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[1])) * sin
    (dynsim_2link_planar_X.Position_CSTATE[2]) - (cos
    (dynsim_2link_planar_X.Position_CSTATE[0]) * cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) - 4.967757600021511E+15 * sin
    (dynsim_2link_planar_X.Position_CSTATE[0]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[1]) / 8.1129638414606682E+31) * cos
    (dynsim_2link_planar_X.Position_CSTATE[2]);
  dynsim_2link_planar_B.d12 = 87.0 * cos(dynsim_2link_planar_X.Position_CSTATE[4]);
  dynsim_2link_planar_B.d13 = 87.0 * sin(dynsim_2link_planar_X.Position_CSTATE[4]);

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynsim_2link_planar_B.msg_g.Position[0] = (((((((dynsim_2link_planar_B.d5 *
    dynsim_2link_planar_B.d4 + dynsim_2link_planar_B.d3 / 8.1129638414606682E+31)
    - dynsim_2link_planar_B.d8 * (4.967757600021511E+15 *
    dynsim_2link_planar_B.d_o) / 8.1129638414606682E+31) +
    (dynsim_2link_planar_B.d7 * dynsim_2link_planar_B.d4 -
     dynsim_2link_planar_B.d5 * dynsim_2link_planar_B.d6) *
    (4.967757600021511E+15 * dynsim_2link_planar_B.d1) / 8.1129638414606682E+31)
    + dynsim_2link_planar_B.d7 * dynsim_2link_planar_B.d6) *
    dynsim_2link_planar_B.d12 / 500.0 + ((((21.0 * dynsim_2link_planar_B.value *
    dynsim_2link_planar_B.value_d / 100.0 - dynsim_2link_planar_B.d5 *
    (8.8199044272416644E+65 * dynsim_2link_planar_B.d4) / 3.2910091146424121E+66)
    - 4.3815147249893147E+81 * dynsim_2link_planar_B.value_l /
    2.6699837949011376E+98) - 1.0432290960045173E+17 *
    dynsim_2link_planar_B.value_l * dynsim_2link_planar_B.d2 /
    8.1129638414606682E+33) - dynsim_2link_planar_B.d8 * (4.3219491120187149E+17
    * dynsim_2link_planar_B.d_o) / 4.0564819207303341E+34)) +
    (dynsim_2link_planar_B.d8 * dynsim_2link_planar_B.d1 +
     dynsim_2link_planar_B.d9 * dynsim_2link_planar_B.d_o) *
    dynsim_2link_planar_B.d13 / 500.0) + dynsim_2link_planar_B.d9 *
    (4.3219491120187149E+17 * dynsim_2link_planar_B.d1) / 4.0564819207303341E+34)
    - dynsim_2link_planar_B.d7 * (8.8199044272416644E+65 *
    dynsim_2link_planar_B.d6) / 3.2910091146424121E+66;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Integrator: '<S1>/Position'

  dynsim_2link_planar_B.d3 = 4.967757600021511E+15 * dynsim_2link_planar_B.value;
  dynsim_2link_planar_B.d5 = dynsim_2link_planar_B.d3 *
    dynsim_2link_planar_B.value_d / 8.1129638414606682E+31 -
    dynsim_2link_planar_B.value_l * dynsim_2link_planar_B.d2;
  dynsim_2link_planar_B.d7 = dynsim_2link_planar_B.d3 * dynsim_2link_planar_B.d2
    / 8.1129638414606682E+31 + dynsim_2link_planar_B.value_d *
    dynsim_2link_planar_B.value_l;
  dynsim_2link_planar_B.d10 = dynsim_2link_planar_B.d7 *
    dynsim_2link_planar_B.d10 / 8.1129638414606682E+31;
  dynsim_2link_planar_B.d11 = dynsim_2link_planar_B.d5 *
    dynsim_2link_planar_B.d11 / 8.1129638414606682E+31;
  dynsim_2link_planar_B.d8 = dynsim_2link_planar_B.d10 -
    dynsim_2link_planar_B.d11;
  dynsim_2link_planar_B.d14 = (4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[0]) * cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) / 8.1129638414606682E+31 - sin
    (dynsim_2link_planar_X.Position_CSTATE[0]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[1])) * sin
    (dynsim_2link_planar_X.Position_CSTATE[2]) + (4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[0]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[1]) / 8.1129638414606682E+31 + cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[0])) * cos
    (dynsim_2link_planar_X.Position_CSTATE[2]);

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Integrator: '<S1>/Position'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynsim_2link_planar_B.msg_g.Velocity[0] = (dynsim_2link_planar_B.d8 *
    dynsim_2link_planar_B.d_o + (dynsim_2link_planar_B.d5 *
    dynsim_2link_planar_B.d4 + dynsim_2link_planar_B.d7 *
    dynsim_2link_planar_B.d6) * dynsim_2link_planar_B.d1) *
    dynsim_2link_planar_B.d12 / 500.0 - (dynsim_2link_planar_B.d8 *
    (4.967757600021511E+15 * sin(dynsim_2link_planar_X.Position_CSTATE[3])) /
    8.1129638414606682E+31 - dynsim_2link_planar_B.d14 * (4.967757600021511E+15 *
    cos(dynsim_2link_planar_X.Position_CSTATE[3])) / 8.1129638414606682E+31) *
    dynsim_2link_planar_B.d13 / 500.0;
  dynsim_2link_planar_B.msg_g.Name[1].Data_SL_Info.CurrentLength = 11U;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Integrator: '<S1>/Position'

  dynsim_2link_planar_B.d8 = ((4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[0]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[1]) / 8.1129638414606682E+31 + cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[0])) * (4.967757600021511E+15 * sin
    (dynsim_2link_planar_X.Position_CSTATE[2])) / 8.1129638414606682E+31 + cos
    (dynsim_2link_planar_X.Position_CSTATE[0])) - (4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[0]) * cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) / 8.1129638414606682E+31 - sin
    (dynsim_2link_planar_X.Position_CSTATE[0]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[1])) * (4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[2])) / 8.1129638414606682E+31;

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Integrator: '<S1>/Position'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynsim_2link_planar_B.msg_g.Position[1] = (((((((4.3815147249893147E+81 *
    dynsim_2link_planar_B.value / 2.6699837949011376E+98 -
    dynsim_2link_planar_B.d7 * (8.8199044272416644E+65 * sin
    (dynsim_2link_planar_X.Position_CSTATE[2])) / 3.2910091146424121E+66) -
    (dynsim_2link_planar_B.d14 * dynsim_2link_planar_B.d_o -
     ((dynsim_2link_planar_B.d10 + dynsim_2link_planar_B.value) -
      dynsim_2link_planar_B.d11) * dynsim_2link_planar_B.d1) *
    dynsim_2link_planar_B.d13 / 500.0) + 1.0432290960045173E+17 *
    dynsim_2link_planar_B.value * dynsim_2link_planar_B.d2 /
    8.1129638414606682E+33) + 21.0 * dynsim_2link_planar_B.value_d *
    dynsim_2link_planar_B.value_l / 100.0) - dynsim_2link_planar_B.d14 *
    (4.3219491120187149E+17 * sin(dynsim_2link_planar_X.Position_CSTATE[3])) /
    4.0564819207303341E+34) - ((dynsim_2link_planar_B.d8 *
    (4.967757600021511E+15 * cos(dynsim_2link_planar_X.Position_CSTATE[3])) /
    8.1129638414606682E+31 + (dynsim_2link_planar_B.d14 * (4.967757600021511E+15
    * sin(dynsim_2link_planar_X.Position_CSTATE[3])) / 8.1129638414606682E+31 +
    (dynsim_2link_planar_B.d3 / 8.1129638414606682E+31 -
     dynsim_2link_planar_B.d7 * dynsim_2link_planar_B.d4))) +
    dynsim_2link_planar_B.d5 * dynsim_2link_planar_B.d6) *
    dynsim_2link_planar_B.d12 / 500.0) - dynsim_2link_planar_B.d8 *
    (4.3219491120187149E+17 * cos(dynsim_2link_planar_X.Position_CSTATE[3])) /
    4.0564819207303341E+34) + dynsim_2link_planar_B.d5 * (8.8199044272416644E+65
    * cos(dynsim_2link_planar_X.Position_CSTATE[2])) / 3.2910091146424121E+66;

  // MATLAB Function: '<Root>/MATLAB Function'
  dynsim_2link_planar_B.value = dynsim_2link_planar_B.d15 +
    dynsim_2link_planar_B.d16;

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Integrator: '<S1>/Position'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynsim_2link_planar_B.msg_g.Velocity[1] = (dynsim_2link_planar_B.value *
    (4.967757600021511E+15 * sin(dynsim_2link_planar_X.Position_CSTATE[3])) /
    8.1129638414606682E+31 + dynsim_2link_planar_B.d9 * (4.967757600021511E+15 *
    cos(dynsim_2link_planar_X.Position_CSTATE[3])) / 8.1129638414606682E+31) *
    dynsim_2link_planar_B.d13 / 500.0 + (dynsim_2link_planar_B.d9 *
    dynsim_2link_planar_B.d1 - dynsim_2link_planar_B.value *
    dynsim_2link_planar_B.d_o) * dynsim_2link_planar_B.d12 / 500.0;
  for (dynsim_2link_planar_B.j_j = 0; dynsim_2link_planar_B.j_j < 11;
       dynsim_2link_planar_B.j_j++) {
    dynsim_2link_planar_B.b.f1[dynsim_2link_planar_B.j_j] =
      h[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.c.f1[dynsim_2link_planar_B.j_j] =
      i[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.d.f1[dynsim_2link_planar_B.j_j] =
      j[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.msg_g.Name[0].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.b.f1[dynsim_2link_planar_B.j_j]);
    dynsim_2link_planar_B.msg_g.Name[1].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.c.f1[dynsim_2link_planar_B.j_j]);
    dynsim_2link_planar_B.msg_g.Name[2].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.d.f1[dynsim_2link_planar_B.j_j]);
  }

  dynsim_2link_planar_B.msg_g.Name[2].Data_SL_Info.CurrentLength = 11U;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Integrator: '<S1>/Position'

  dynsim_2link_planar_B.value = (4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) * cos
    (dynsim_2link_planar_X.Position_CSTATE[2]) / 8.1129638414606682E+31 -
    4.967757600021511E+15 * sin(dynsim_2link_planar_X.Position_CSTATE[1]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[2]) / 8.1129638414606682E+31) +
    6.123233995736766E-17;
  dynsim_2link_planar_B.value_l = cos(dynsim_2link_planar_X.Position_CSTATE[1]) *
    sin(dynsim_2link_planar_X.Position_CSTATE[2]) + cos
    (dynsim_2link_planar_X.Position_CSTATE[2]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[1]);

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Integrator: '<S1>/Position'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynsim_2link_planar_B.msg_g.Position[2] = ((((((67.0 *
    dynsim_2link_planar_B.d2 * dynsim_2link_planar_B.d4 / 250.0 - 67.0 *
    dynsim_2link_planar_B.value_d * dynsim_2link_planar_B.d6 / 250.0) - 21.0 *
    dynsim_2link_planar_B.d2 / 100.0) - (((((dynsim_2link_planar_B.d17 *
    dynsim_2link_planar_B.d6 / 8.1129638414606682E+31 - 4.967757600021511E+15 *
    dynsim_2link_planar_B.d2 * dynsim_2link_planar_B.d4 / 8.1129638414606682E+31)
    + 6.123233995736766E-17) * (4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[3])) / 8.1129638414606682E+31 +
    (dynsim_2link_planar_B.d2 * dynsim_2link_planar_B.d4 -
     dynsim_2link_planar_B.value_d * dynsim_2link_planar_B.d6)) -
    (dynsim_2link_planar_B.value_d * dynsim_2link_planar_B.d4 +
     dynsim_2link_planar_B.d6 * dynsim_2link_planar_B.d2) *
    (4.967757600021511E+15 * sin(dynsim_2link_planar_X.Position_CSTATE[3])) /
    8.1129638414606682E+31) + 3.749399456654644E-33) * dynsim_2link_planar_B.d12
    / 500.0) - dynsim_2link_planar_B.value * (3.376522743764621E+15 *
    dynsim_2link_planar_B.d_o) / 3.1691265005705735E+32) +
    dynsim_2link_planar_B.value_l * (3.376522743764621E+15 *
    dynsim_2link_planar_B.d1) / 3.1691265005705735E+32) +
    (dynsim_2link_planar_B.value * dynsim_2link_planar_B.d1 +
     dynsim_2link_planar_B.value_l * dynsim_2link_planar_B.d_o) *
    dynsim_2link_planar_B.d13 / 500.0) + 0.337;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Integrator: '<S1>/Position'

  dynsim_2link_planar_B.value = 4.967757600021511E+15 * cos
    (dynsim_2link_planar_X.Position_CSTATE[1]) * cos
    (dynsim_2link_planar_X.Position_CSTATE[2]) / 8.1129638414606682E+31 -
    4.967757600021511E+15 * sin(dynsim_2link_planar_X.Position_CSTATE[1]) * sin
    (dynsim_2link_planar_X.Position_CSTATE[2]) / 8.1129638414606682E+31;

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Integrator: '<S1>/Position'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynsim_2link_planar_B.msg_g.Velocity[2] = (dynsim_2link_planar_B.value *
    (4.967757600021511E+15 * sin(dynsim_2link_planar_X.Position_CSTATE[3])) /
    8.1129638414606682E+31 + dynsim_2link_planar_B.value_l *
    (4.967757600021511E+15 * cos(dynsim_2link_planar_X.Position_CSTATE[3])) /
    8.1129638414606682E+31) * dynsim_2link_planar_B.d13 / 500.0 -
    (dynsim_2link_planar_B.value * dynsim_2link_planar_B.d_o -
     dynsim_2link_planar_B.value_l * dynsim_2link_planar_B.d1) *
    dynsim_2link_planar_B.d12 / 500.0;

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S10>/SinkBlock'
  Pub_dynsim_2link_planar_124.publish(&dynsim_2link_planar_B.msg_g);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Constant: '<S5>/Constant'
  //   Integrator: '<S1>/Position'

  dynsim_2link_planar_B.msg_g = dynsim_2link_planar_P.Constant_Value;
  dynsim_2link_planar_B.msg_g.Header.Stamp.Sec = dynsim_2link_planar_B.k;
  dynsim_2link_planar_B.msg_g.Header.Stamp.Nsec = dynsim_2link_planar_B.j;
  dynsim_2link_planar_B.msg_g.Name_SL_Info.CurrentLength = 6U;
  dynsim_2link_planar_B.msg_g.Position_SL_Info.CurrentLength = 6U;
  dynsim_2link_planar_B.msg_g.Velocity_SL_Info.CurrentLength = 6U;
  dynsim_2link_planar_B.msg_g.Name[0].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg_g.Position[0] =
    dynsim_2link_planar_X.Position_CSTATE[0];
  dynsim_2link_planar_B.msg_g.Velocity[0] = dynsim_2link_planar_B.Velocity[0];
  dynsim_2link_planar_B.msg_g.Name[1].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg_g.Position[1] =
    dynsim_2link_planar_X.Position_CSTATE[1];
  dynsim_2link_planar_B.msg_g.Velocity[1] = dynsim_2link_planar_B.Velocity[1];
  dynsim_2link_planar_B.msg_g.Name[2].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg_g.Position[2] =
    dynsim_2link_planar_X.Position_CSTATE[2];
  dynsim_2link_planar_B.msg_g.Velocity[2] = dynsim_2link_planar_B.Velocity[2];
  dynsim_2link_planar_B.msg_g.Name[3].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg_g.Position[3] =
    dynsim_2link_planar_X.Position_CSTATE[3];
  dynsim_2link_planar_B.msg_g.Velocity[3] = dynsim_2link_planar_B.Velocity[3];
  dynsim_2link_planar_B.msg_g.Name[4].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg_g.Position[4] =
    dynsim_2link_planar_X.Position_CSTATE[4];
  dynsim_2link_planar_B.msg_g.Velocity[4] = dynsim_2link_planar_B.Velocity[4];
  for (dynsim_2link_planar_B.j_j = 0; dynsim_2link_planar_B.j_j < 11;
       dynsim_2link_planar_B.j_j++) {
    dynsim_2link_planar_B.b.f1[dynsim_2link_planar_B.j_j] =
      h[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.c.f1[dynsim_2link_planar_B.j_j] =
      i[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.d.f1[dynsim_2link_planar_B.j_j] =
      j[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.e.f1[dynsim_2link_planar_B.j_j] =
      k[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.f.f1[dynsim_2link_planar_B.j_j] =
      l[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.g.f1[dynsim_2link_planar_B.j_j] =
      m[dynsim_2link_planar_B.j_j];
    dynsim_2link_planar_B.msg_g.Name[0].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.b.f1[dynsim_2link_planar_B.j_j]);
    dynsim_2link_planar_B.msg_g.Name[1].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.c.f1[dynsim_2link_planar_B.j_j]);
    dynsim_2link_planar_B.msg_g.Name[2].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.d.f1[dynsim_2link_planar_B.j_j]);
    dynsim_2link_planar_B.msg_g.Name[3].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.e.f1[dynsim_2link_planar_B.j_j]);
    dynsim_2link_planar_B.msg_g.Name[4].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.f.f1[dynsim_2link_planar_B.j_j]);
    dynsim_2link_planar_B.msg_g.Name[5].Data[dynsim_2link_planar_B.j_j] =
      static_cast<uint8_T>(dynsim_2link_planar_B.g.f1[dynsim_2link_planar_B.j_j]);
  }

  dynsim_2link_planar_B.msg_g.Name[5].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg_g.Position[5] =
    dynsim_2link_planar_X.Position_CSTATE[5];
  dynsim_2link_planar_B.msg_g.Velocity[5] = dynsim_2link_planar_B.Velocity[5];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_dynsim_2link_planar_22.publish(&dynsim_2link_planar_B.msg_g);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (dynsim_2link_planar_B.vNum < 0.0) {
    dynsim_2link_planar_B.k = ceil(dynsim_2link_planar_B.vNum);
  } else {
    dynsim_2link_planar_B.k = floor(dynsim_2link_planar_B.vNum);
  }

  dynsim_2link_planar_B.msg_l.Clock_.Sec = dynsim_2link_planar_B.k;
  dynsim_2link_planar_B.j = (dynsim_2link_planar_B.vNum -
    dynsim_2link_planar_B.k) * 1.0E+9;
  if (dynsim_2link_planar_B.j < 0.0) {
    dynsim_2link_planar_B.msg_l.Clock_.Nsec = ceil(dynsim_2link_planar_B.j);
  } else {
    dynsim_2link_planar_B.msg_l.Clock_.Nsec = floor(dynsim_2link_planar_B.j);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_dynsim_2link_planar_50.publish(&dynsim_2link_planar_B.msg_l);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    // Update for Integrator: '<S1>/Position'
    dynsim_2link_planar_DW.Position_IWORK = 0;

    // Update for Integrator: '<S1>/Velocity'
    dynsim_2link_planar_DW.Velocity_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    rt_ertODEUpdateContinuousStates(&dynsim_2link_planar_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++dynsim_2link_planar_M->Timing.clockTick0;
    dynsim_2link_planar_M->Timing.t[0] = rtsiGetSolverStopTime
      (&dynsim_2link_planar_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.05s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.05, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      dynsim_2link_planar_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void dynsim_2link_planar_derivatives(void)
{
  int32_T i;
  XDot_dynsim_2link_planar_T *_rtXdot;
  _rtXdot = ((XDot_dynsim_2link_planar_T *) dynsim_2link_planar_M->derivs);
  for (i = 0; i < 6; i++) {
    // Derivatives for Integrator: '<S1>/Position'
    _rtXdot->Position_CSTATE[i] = dynsim_2link_planar_B.Velocity[i];

    // Derivatives for Integrator: '<S1>/Velocity'
    _rtXdot->Velocity_CSTATE[i] = dynsim_2link_planar_B.MATLABSystem[i];
  }
}

// Model initialize function
void dynsim_2link_planar_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&dynsim_2link_planar_M->solverInfo,
                          &dynsim_2link_planar_M->Timing.simTimeStep);
    rtsiSetTPtr(&dynsim_2link_planar_M->solverInfo, &rtmGetTPtr
                (dynsim_2link_planar_M));
    rtsiSetStepSizePtr(&dynsim_2link_planar_M->solverInfo,
                       &dynsim_2link_planar_M->Timing.stepSize0);
    rtsiSetdXPtr(&dynsim_2link_planar_M->solverInfo,
                 &dynsim_2link_planar_M->derivs);
    rtsiSetContStatesPtr(&dynsim_2link_planar_M->solverInfo, (real_T **)
                         &dynsim_2link_planar_M->contStates);
    rtsiSetNumContStatesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&dynsim_2link_planar_M->solverInfo,
                          (&rtmGetErrorStatus(dynsim_2link_planar_M)));
    rtsiSetRTModelPtr(&dynsim_2link_planar_M->solverInfo, dynsim_2link_planar_M);
  }

  rtsiSetSimTimeStep(&dynsim_2link_planar_M->solverInfo, MAJOR_TIME_STEP);
  dynsim_2link_planar_M->intgData.y = dynsim_2link_planar_M->odeY;
  dynsim_2link_planar_M->intgData.f[0] = dynsim_2link_planar_M->odeF[0];
  dynsim_2link_planar_M->intgData.f[1] = dynsim_2link_planar_M->odeF[1];
  dynsim_2link_planar_M->intgData.f[2] = dynsim_2link_planar_M->odeF[2];
  dynsim_2link_planar_M->contStates = ((X_dynsim_2link_planar_T *)
    &dynsim_2link_planar_X);
  rtsiSetSolverData(&dynsim_2link_planar_M->solverInfo, static_cast<void *>
                    (&dynsim_2link_planar_M->intgData));
  rtsiSetSolverName(&dynsim_2link_planar_M->solverInfo,"ode3");
  rtmSetTPtr(dynsim_2link_planar_M, &dynsim_2link_planar_M->Timing.tArray[0]);
  dynsim_2link_planar_M->Timing.stepSize0 = 0.05;
  rtmSetFirstInitCond(dynsim_2link_planar_M, 1);

  {
    char_T tmp[7];
    int32_T i;
    static const char_T tmp_0[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_1[17] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 's', 't', 'a', 't', 'e', 's' };

    static const char_T tmp_2[17] = { '/', 'e', 'd', 'o', '/', 'j', 'o', 'i',
      'n', 't', '_', 's', 't', 'a', 't', 'e', 's' };

    static const char_T tmp_3[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_4[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '1',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_5[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '2',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_6[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '3',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_7[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '4',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_8[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '5',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_9[31] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '6',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_a[32] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', 'v',
      '1', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_b[33] = { '/', 'd', 'y', 'n', 'n', 's', 'i', 'm',
      '_', '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q',
      'v', '2', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_c[32] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', 'v',
      '3', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_d[32] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', 'v',
      '4', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_e[33] = { '/', 'd', 'y', 'n', 'n', 's', 'i', 'm',
      '_', '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q',
      'v', '5', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_f[32] = { '/', 'd', 'y', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', 'v',
      '5', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    // InitializeConditions for Integrator: '<S1>/Position' incorporates:
    //   Integrator: '<S1>/Velocity'

    if (rtmIsFirstInitCond(dynsim_2link_planar_M)) {
      dynsim_2link_planar_X.Position_CSTATE[0] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[1] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[2] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[3] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[4] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[5] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[0] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[1] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[2] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[3] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[4] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[5] = 0.0;
    }

    dynsim_2link_planar_DW.Position_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S1>/Position'

    // InitializeConditions for Integrator: '<S1>/Velocity'
    dynsim_2link_planar_DW.Velocity_IWORK = 1;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S14>/Out1'
    dynsim_2link_planar_B.In1 = dynsim_2link_planar_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem'

    // Start for MATLABSystem: '<S11>/SourceBlock'
    dynsim_2link_planar_DW.obj_pp.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_pp.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynsim_2link_planar_B.cv4[i] = tmp_0[i];
    }

    dynsim_2link_planar_B.cv4[13] = '\x00';
    Sub_dynsim_2link_planar_16.createSubscriber(dynsim_2link_planar_B.cv4, 1);
    dynsim_2link_planar_DW.obj_pp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    dynsim_2link_planar_DW.obj_a.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      dynsim_2link_planar_B.cv3[i] = tmp_1[i];
    }

    dynsim_2link_planar_B.cv3[17] = '\x00';
    Pub_dynsim_2link_planar_124.createPublisher(dynsim_2link_planar_B.cv3, 1);
    dynsim_2link_planar_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    dynsim_2link_planar_DW.obj_nr.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      dynsim_2link_planar_B.cv3[i] = tmp_2[i];
    }

    dynsim_2link_planar_B.cv3[17] = '\x00';
    Pub_dynsim_2link_planar_22.createPublisher(dynsim_2link_planar_B.cv3, 1);
    dynsim_2link_planar_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    dynsim_2link_planar_DW.obj_f.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_3[i];
    }

    tmp[6] = '\x00';
    Pub_dynsim_2link_planar_50.createPublisher(tmp, 1);
    dynsim_2link_planar_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S13>/Get Parameter'
    dynsim_2link_planar_DW.obj_p.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_2link_planar_B.cv2[i] = tmp_4[i];
    }

    dynsim_2link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_2link_planar_112.initialize(dynsim_2link_planar_B.cv2);
    ParamGet_dynsim_2link_planar_112.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_112.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter'

    // Start for MATLABSystem: '<S13>/Get Parameter1'
    dynsim_2link_planar_DW.obj_n.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_2link_planar_B.cv2[i] = tmp_5[i];
    }

    dynsim_2link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_2link_planar_113.initialize(dynsim_2link_planar_B.cv2);
    ParamGet_dynsim_2link_planar_113.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_113.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter1'

    // Start for MATLABSystem: '<S13>/Get Parameter4'
    dynsim_2link_planar_DW.obj_d.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_2link_planar_B.cv2[i] = tmp_6[i];
    }

    dynsim_2link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_2link_planar_129.initialize(dynsim_2link_planar_B.cv2);
    ParamGet_dynsim_2link_planar_129.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_129.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter4'

    // Start for MATLABSystem: '<S13>/Get Parameter6'
    dynsim_2link_planar_DW.obj_d5.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_d5.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_2link_planar_B.cv2[i] = tmp_7[i];
    }

    dynsim_2link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_2link_planar_131.initialize(dynsim_2link_planar_B.cv2);
    ParamGet_dynsim_2link_planar_131.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_131.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_d5.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter6'

    // Start for MATLABSystem: '<S13>/Get Parameter7'
    dynsim_2link_planar_DW.obj_l.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_2link_planar_B.cv2[i] = tmp_8[i];
    }

    dynsim_2link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_2link_planar_132.initialize(dynsim_2link_planar_B.cv2);
    ParamGet_dynsim_2link_planar_132.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_132.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter7'

    // Start for MATLABSystem: '<S13>/Get Parameter8'
    dynsim_2link_planar_DW.obj_nb.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_nb.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      dynsim_2link_planar_B.cv2[i] = tmp_9[i];
    }

    dynsim_2link_planar_B.cv2[31] = '\x00';
    ParamGet_dynsim_2link_planar_133.initialize(dynsim_2link_planar_B.cv2);
    ParamGet_dynsim_2link_planar_133.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_133.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_nb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter8'

    // Start for MATLABSystem: '<S13>/Get Parameter2'
    dynsim_2link_planar_DW.obj_h.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 32; i++) {
      dynsim_2link_planar_B.cv1[i] = tmp_a[i];
    }

    dynsim_2link_planar_B.cv1[32] = '\x00';
    ParamGet_dynsim_2link_planar_117.initialize(dynsim_2link_planar_B.cv1);
    ParamGet_dynsim_2link_planar_117.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_117.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter2'

    // Start for MATLABSystem: '<S13>/Get Parameter3'
    dynsim_2link_planar_DW.obj_b.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 33; i++) {
      dynsim_2link_planar_B.cv[i] = tmp_b[i];
    }

    dynsim_2link_planar_B.cv[33] = '\x00';
    ParamGet_dynsim_2link_planar_118.initialize(dynsim_2link_planar_B.cv);
    ParamGet_dynsim_2link_planar_118.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_118.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter3'

    // Start for MATLABSystem: '<S13>/Get Parameter5'
    dynsim_2link_planar_DW.obj_o.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 32; i++) {
      dynsim_2link_planar_B.cv1[i] = tmp_c[i];
    }

    dynsim_2link_planar_B.cv1[32] = '\x00';
    ParamGet_dynsim_2link_planar_130.initialize(dynsim_2link_planar_B.cv1);
    ParamGet_dynsim_2link_planar_130.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_130.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter5'

    // Start for MATLABSystem: '<S13>/Get Parameter9'
    dynsim_2link_planar_DW.obj_pd.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_pd.isInitialized = 1;
    for (i = 0; i < 32; i++) {
      dynsim_2link_planar_B.cv1[i] = tmp_d[i];
    }

    dynsim_2link_planar_B.cv1[32] = '\x00';
    ParamGet_dynsim_2link_planar_134.initialize(dynsim_2link_planar_B.cv1);
    ParamGet_dynsim_2link_planar_134.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_134.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_pd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter9'

    // Start for MATLABSystem: '<S13>/Get Parameter10'
    dynsim_2link_planar_DW.obj_e.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 33; i++) {
      dynsim_2link_planar_B.cv[i] = tmp_e[i];
    }

    dynsim_2link_planar_B.cv[33] = '\x00';
    ParamGet_dynsim_2link_planar_135.initialize(dynsim_2link_planar_B.cv);
    ParamGet_dynsim_2link_planar_135.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_135.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter10'

    // Start for MATLABSystem: '<S13>/Get Parameter11'
    dynsim_2link_planar_DW.obj_ez.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_ez.isInitialized = 1;
    for (i = 0; i < 32; i++) {
      dynsim_2link_planar_B.cv1[i] = tmp_f[i];
    }

    dynsim_2link_planar_B.cv1[32] = '\x00';
    ParamGet_dynsim_2link_planar_136.initialize(dynsim_2link_planar_B.cv1);
    ParamGet_dynsim_2link_planar_136.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_136.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_ez.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter11'
    emxInitStruct_robotics_slmanip_(&dynsim_2link_planar_DW.obj);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_1);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_12);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_11);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_10);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_9);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_8);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_7);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_6);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_5);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_4);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_3);
    emxInitStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_2);

    // Start for MATLABSystem: '<S12>/MATLAB System'
    dynsim_2link_planar_DW.obj.isInitialized = 0;
    dynsim_2link_planar_DW.obj.isInitialized = 1;
    dyn_RigidBodyTree_RigidBodyTree(&dynsim_2link_planar_DW.obj.TreeInternal,
      &dynsim_2link_planar_DW.gobj_2, &dynsim_2link_planar_DW.gobj_4,
      &dynsim_2link_planar_DW.gobj_5, &dynsim_2link_planar_DW.gobj_6,
      &dynsim_2link_planar_DW.gobj_7, &dynsim_2link_planar_DW.gobj_3);
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(dynsim_2link_planar_M)) {
    rtmSetFirstInitCond(dynsim_2link_planar_M, 0);
  }
}

// Model terminate function
void dynsim_2link_planar_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S11>/SourceBlock'
  matlabCodegenHa_awtnueqcp3y53m2(&dynsim_2link_planar_DW.obj_pp);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S13>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_p);

  // Terminate for MATLABSystem: '<S13>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_n);

  // Terminate for MATLABSystem: '<S13>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_d);

  // Terminate for MATLABSystem: '<S13>/Get Parameter6'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_d5);

  // Terminate for MATLABSystem: '<S13>/Get Parameter7'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_l);

  // Terminate for MATLABSystem: '<S13>/Get Parameter8'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_nb);

  // Terminate for MATLABSystem: '<S13>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_h);

  // Terminate for MATLABSystem: '<S13>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_b);

  // Terminate for MATLABSystem: '<S13>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_o);

  // Terminate for MATLABSystem: '<S13>/Get Parameter9'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_pd);

  // Terminate for MATLABSystem: '<S13>/Get Parameter10'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_e);

  // Terminate for MATLABSystem: '<S13>/Get Parameter11'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_ez);
  emxFreeStruct_robotics_slmanip_(&dynsim_2link_planar_DW.obj);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_1);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_12);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_11);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_10);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_9);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_8);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_7);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_6);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_5);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_4);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_3);
  emxFreeStruct_l_robotics_manip_(&dynsim_2link_planar_DW.gobj_2);

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandl_awtnueqcp3y5(&dynsim_2link_planar_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandl_awtnueqcp3y5(&dynsim_2link_planar_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandl_awtnueqcp3y5(&dynsim_2link_planar_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
