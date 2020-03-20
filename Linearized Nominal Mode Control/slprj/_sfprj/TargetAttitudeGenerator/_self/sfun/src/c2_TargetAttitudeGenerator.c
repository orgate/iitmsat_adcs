/* Include files */

#include "blascompat32.h"
#include "TargetAttitudeGenerator_sfun.h"
#include "c2_TargetAttitudeGenerator.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance.instanceNumber)
#include "TargetAttitudeGenerator_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c2_IN_NO_ACTIVE_CHILD          (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc2_TargetAttitudeGeneratorInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c2_TargetAttitudeGenerator(void);
static void initialize_params_c2_TargetAttitudeGenerator(void);
static void enable_c2_TargetAttitudeGenerator(void);
static void disable_c2_TargetAttitudeGenerator(void);
static void c2_update_debugger_state_c2_TargetAttitudeGenerator(void);
static const mxArray *get_sim_state_c2_TargetAttitudeGenerator(void);
static void set_sim_state_c2_TargetAttitudeGenerator(const mxArray *c2_st);
static void finalize_c2_TargetAttitudeGenerator(void);
static void sf_c2_TargetAttitudeGenerator(void);
static void c2_c2_TargetAttitudeGenerator(void);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static real_T c2_mrdivide(real_T c2_A, real_T c2_B);
static void c2_eml_warning(void);
static void c2_LatLong(real_T c2_r_ECEF[3], real_T *c2_lat, real_T *c2_long);
static real_T c2_mpower(real_T c2_a);
static void c2_eml_error(void);
static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_d_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_e_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_f_sf_marshall(void *c2_chartInstance, void *c2_u);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[43]);
static const mxArray *c2_emlrt_marshallOut(uint8_T c2_u);
static void c2_emlrt_marshallIn(const mxArray *c2_TA_at_t, char *c2_name, real_T
  c2_y[3]);
static uint8_T c2_b_emlrt_marshallIn(const mxArray
  *c2_b_is_active_c2_TargetAttitudeGenerator, char *c2_name);
static void init_io_bus_offset(void);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c2_TargetAttitudeGenerator(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c2_is_active_c2_TargetAttitudeGenerator = 0U;
}

static void initialize_params_c2_TargetAttitudeGenerator(void)
{
}

static void enable_c2_TargetAttitudeGenerator(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void disable_c2_TargetAttitudeGenerator(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void c2_update_debugger_state_c2_TargetAttitudeGenerator(void)
{
}

static const mxArray *get_sim_state_c2_TargetAttitudeGenerator(void)
{
  const mxArray *c2_st = NULL;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[3];
  const mxArray *c2_b_y = NULL;
  real_T (*c2_TA_at_t)[3];
  c2_TA_at_t = (real_T (*)[3])ssGetOutputPortSignal(chartInstance.S, 1);
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2));
  for (c2_i0 = 0; c2_i0 < 3; c2_i0 = c2_i0 + 1) {
    c2_u[c2_i0] = (*c2_TA_at_t)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 1U, 0U, 1, 3));
  sf_mex_setcell(c2_y, 0, c2_b_y);
  sf_mex_setcell(c2_y, 1, c2_emlrt_marshallOut
                 (chartInstance.c2_is_active_c2_TargetAttitudeGenerator));
  sf_mex_assign(&c2_st, c2_y);
  return c2_st;
}

static void set_sim_state_c2_TargetAttitudeGenerator(const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[3];
  int32_T c2_i1;
  real_T (*c2_TA_at_t)[3];
  c2_TA_at_t = (real_T (*)[3])ssGetOutputPortSignal(chartInstance.S, 1);
  chartInstance.c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(sf_mex_dup(sf_mex_getcell(c2_u, 0)), "TA_at_t", c2_dv0);
  for (c2_i1 = 0; c2_i1 < 3; c2_i1 = c2_i1 + 1) {
    (*c2_TA_at_t)[c2_i1] = c2_dv0[c2_i1];
  }

  chartInstance.c2_is_active_c2_TargetAttitudeGenerator = c2_b_emlrt_marshallIn
    (sf_mex_dup(sf_mex_getcell(c2_u, 1)),
     "is_active_c2_TargetAttitudeGenerator");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_TargetAttitudeGenerator();
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_TargetAttitudeGenerator(void)
{
}

static void sf_c2_TargetAttitudeGenerator(void)
{
  int32_T c2_i2;
  uint8_T c2_previousEvent;
  real_T *c2_t;
  real_T (*c2_TA_at_t)[3];
  c2_t = (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
  c2_TA_at_t = (real_T (*)[3])ssGetOutputPortSignal(chartInstance.S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,0);
  _SFD_DATA_RANGE_CHECK(*c2_t, 0U);
  for (c2_i2 = 0; c2_i2 < 3; c2_i2 = c2_i2 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_TA_at_t)[c2_i2], 1U);
  }

  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c2_c2_TargetAttitudeGenerator();
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_TargetAttitudeGeneratorMachineNumber_,
    chartInstance.chartNumber, chartInstance.
    instanceNumber);
}

static void c2_c2_TargetAttitudeGenerator(void)
{
  real_T c2_t;
  real_T c2_nargout = 1.0;
  real_T c2_nargin = 1.0;
  const mxArray *c2_a = NULL;
  const mxArray *c2_B_ECEF_sph = NULL;
  const mxArray *c2_gh = NULL;
  const mxArray *c2_r_ECI = NULL;
  real_T c2_r_ECEF[180];
  real_T c2_long[60];
  real_T c2_lat[60];
  real_T c2_dip[60];
  real_T c2_inclination;
  real_T c2_oo;
  real_T c2_TA[540];
  real_T c2_num;
  real_T c2_omega;
  real_T c2_Omega;
  real_T c2_altitude;
  real_T c2_time_step;
  real_T c2_t_sim;
  real_T c2_TA_at_t[3];
  int32_T c2_i3;
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  real_T c2_b_t;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_b;
  real_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_b_b;
  real_T c2_f_u;
  const mxArray *c2_f_y = NULL;
  const mxArray *c2_mtimes;
  real_T c2_dv1[3];
  int32_T c2_i10;
  real_T c2_g_y[3];
  int32_T c2_c_t;
  int32_T c2_i11;
  int32_T c2_d_t;
  int32_T c2_i12;
  real_T c2_b_r_ECEF[3];
  real_T c2_d0;
  real_T c2_d1;
  real_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  real_T c2_j_u;
  const mxArray *c2_k_y = NULL;
  real_T c2_k_u;
  const mxArray *c2_l_y = NULL;
  const mxArray *c2_b_B_ECEF_sph;
  real_T c2_dv2[3];
  int32_T c2_i13;
  int32_T c2_i14;
  real_T *c2_e_t;
  real_T (*c2_b_TA_at_t)[3];
  c2_e_t = (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
  c2_b_TA_at_t = (real_T (*)[3])ssGetOutputPortSignal(chartInstance.S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  c2_t = *c2_e_t;
  sf_debug_symbol_scope_push(21U, 0U);
  sf_debug_symbol_scope_add("nargout", &c2_nargout, c2_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c2_nargin, c2_sf_marshall);
  sf_debug_symbol_scope_add("a", &c2_a, c2_f_sf_marshall);
  sf_debug_symbol_scope_add("B_ECEF_sph", &c2_B_ECEF_sph, c2_f_sf_marshall);
  sf_debug_symbol_scope_add("gh", &c2_gh, c2_f_sf_marshall);
  sf_debug_symbol_scope_add("r_ECI", &c2_r_ECI, c2_f_sf_marshall);
  sf_debug_symbol_scope_add("r_ECEF", &c2_r_ECEF, c2_e_sf_marshall);
  sf_debug_symbol_scope_add("long", &c2_long, c2_d_sf_marshall);
  sf_debug_symbol_scope_add("lat", &c2_lat, c2_d_sf_marshall);
  sf_debug_symbol_scope_add("dip", &c2_dip, c2_d_sf_marshall);
  sf_debug_symbol_scope_add("inclination", &c2_inclination, c2_sf_marshall);
  sf_debug_symbol_scope_add("oo", &c2_oo, c2_sf_marshall);
  sf_debug_symbol_scope_add("TA", &c2_TA, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("num", &c2_num, c2_sf_marshall);
  sf_debug_symbol_scope_add("omega", &c2_omega, c2_sf_marshall);
  sf_debug_symbol_scope_add("Omega", &c2_Omega, c2_sf_marshall);
  sf_debug_symbol_scope_add("altitude", &c2_altitude, c2_sf_marshall);
  sf_debug_symbol_scope_add("time_step", &c2_time_step, c2_sf_marshall);
  sf_debug_symbol_scope_add("t_sim", &c2_t_sim, c2_sf_marshall);
  sf_debug_symbol_scope_add("TA_at_t", &c2_TA_at_t, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("t", &c2_t, c2_sf_marshall);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0,3);
  _SFD_EML_CALL(0,4);
  _SFD_EML_CALL(0,5);
  _SFD_EML_CALL(0,6);
  _SFD_EML_CALL(0,7);
  _SFD_EML_CALL(0,8);
  _SFD_EML_CALL(0,9);
  _SFD_EML_CALL(0,12);
  for (c2_i3 = 0; c2_i3 < 3; c2_i3 = c2_i3 + 1) {
    c2_TA_at_t[c2_i3] = 0.0;
  }

  _SFD_EML_CALL(0,13);
  c2_t_sim = 6000.0;

  /* seconds */
  _SFD_EML_CALL(0,14);
  c2_time_step = 100.0;

  /* seconds */
  /* orbit parameters */
  _SFD_EML_CALL(0,17);
  c2_altitude = 8.0E+005;
  _SFD_EML_CALL(0,18);
  c2_Omega = c2_mrdivide(8.0943134719741010E+002, 180.0);

  /* RAAN */
  _SFD_EML_CALL(0,19);
  c2_omega = 0.0;

  /* argument of perigee. Has no significance for a circular orbit.  */
  /* If an elliptical orbit is required, modifications must be made to the */
  /* function ECIOrbitModel. */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  _SFD_EML_CALL(0,23);
  c2_num = 60.0;
  _SFD_EML_CALL(0,24);
  for (c2_i4 = 0; c2_i4 < 540; c2_i4 = c2_i4 + 1) {
    c2_TA[c2_i4] = 0.0;
  }

  _SFD_EML_CALL(0,25);
  c2_oo = 1.0;
  _SFD_EML_CALL(0,26);
  c2_inclination = c2_mrdivide(2.8274333882308139E+002, 180.0);
  _SFD_EML_CALL(0,27);
  for (c2_i5 = 0; c2_i5 < 60; c2_i5 = c2_i5 + 1) {
    c2_dip[c2_i5] = 0.0;
  }

  _SFD_EML_CALL(0,28);
  for (c2_i6 = 0; c2_i6 < 540; c2_i6 = c2_i6 + 1) {
    c2_TA[c2_i6] = 0.0;
  }

  _SFD_EML_CALL(0,29);
  for (c2_i7 = 0; c2_i7 < 60; c2_i7 = c2_i7 + 1) {
    c2_lat[c2_i7] = 0.0;
  }

  _SFD_EML_CALL(0,30);
  for (c2_i8 = 0; c2_i8 < 60; c2_i8 = c2_i8 + 1) {
    c2_long[c2_i8] = 0.0;
  }

  _SFD_EML_CALL(0,31);
  for (c2_i9 = 0; c2_i9 < 180; c2_i9 = c2_i9 + 1) {
    c2_r_ECEF[c2_i9] = 0.0;
  }

  c2_t = 1.0;
  c2_b_t = 1.0;
  while (c2_b_t <= 59.0) {
    c2_t = c2_b_t;
    CV_EML_FOR(0, 0, 1);
    _SFD_EML_CALL(0,33);
    c2_u = c2_altitude;
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0));
    c2_b_u = c2_omega;
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
    c2_c_u = c2_inclination;
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0));
    c2_d_u = c2_Omega;
    c2_d_y = NULL;
    sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0));
    c2_b = c2_t;
    c2_e_u = 100.0 * c2_b;
    c2_e_y = NULL;
    sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0));
    sf_mex_assign(&c2_r_ECI, sf_mex_call_debug("ECIOrbitModel", 1U, 5U, 14,
      c2_e_y, 14, c2_d_y, 14, c2_c_y, 14, c2_b_y, 14, c2_y));
    _SFD_EML_CALL(0,34);
    c2_b_b = c2_t;
    c2_f_u = 100.0 * c2_b_b;
    c2_f_y = NULL;
    sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0));
    c2_mtimes = sf_mex_call_debug("mtimes", 1U, 2U, 14, sf_mex_call_debug(
      "DCMECItoECEF", 1U, 1U, 14, c2_f_y), 14, sf_mex_dup(c2_r_ECI));
    sf_mex_import("mtimes", sf_mex_dup(c2_mtimes), &c2_dv1, 0, 0, 0U, 1, 0U, 1,
                  3);
    for (c2_i10 = 0; c2_i10 < 3; c2_i10 = c2_i10 + 1) {
      c2_g_y[c2_i10] = c2_dv1[c2_i10];
    }

    sf_mex_destroy(&c2_mtimes);
    c2_c_t = _SFD_EML_ARRAY_BOUNDS_CHECK("r_ECEF", (int32_T)_SFD_INTEGER_CHECK(
      "t", c2_t), 1, 60, 2, 0) - 1;
    for (c2_i11 = 0; c2_i11 < 3; c2_i11 = c2_i11 + 1) {
      c2_r_ECEF[c2_i11 + 3 * c2_c_t] = c2_g_y[c2_i11];
    }

    _SFD_EML_CALL(0,35);
    c2_d_t = _SFD_EML_ARRAY_BOUNDS_CHECK("r_ECEF", (int32_T)_SFD_INTEGER_CHECK(
      "t", c2_t), 1, 60, 2, 0) - 1;
    for (c2_i12 = 0; c2_i12 < 3; c2_i12 = c2_i12 + 1) {
      c2_b_r_ECEF[c2_i12] = c2_r_ECEF[c2_i12 + 3 * c2_d_t];
    }

    c2_LatLong(c2_b_r_ECEF, &c2_d1, &c2_d0);
    c2_lat[_SFD_EML_ARRAY_BOUNDS_CHECK("lat", (int32_T)_SFD_INTEGER_CHECK("t",
      c2_t), 1, 60, 1, 0) - 1] = c2_d1;
    c2_long[_SFD_EML_ARRAY_BOUNDS_CHECK("long", (int32_T)_SFD_INTEGER_CHECK("t",
      c2_t), 1, 60, 1, 0) - 1] = c2_d0;

    /*          %initializing IGRF */
    /*          */
    _SFD_EML_CALL(0,39);
    c2_g_u = 1.0;
    c2_h_y = NULL;
    sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0));
    sf_mex_assign(&c2_gh, sf_mex_call_debug("GetIGRF11_Coefficients", 1U, 1U, 14,
      c2_h_y));

    /*           */
    /*          %Finding B, dip, and target attitude (here, target attitude assumes */
    /*          %the Cones-Not-Intersecting case.) */
    _SFD_EML_CALL(0,43);
    c2_h_u = c2_long[_SFD_EML_ARRAY_BOUNDS_CHECK("long", (int32_T)
      _SFD_INTEGER_CHECK("t", c2_t), 1, 60, 1, 0) - 1];
    c2_i_y = NULL;
    sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0));
    c2_i_u = c2_lat[_SFD_EML_ARRAY_BOUNDS_CHECK("lat", (int32_T)
      _SFD_INTEGER_CHECK("t", c2_t), 1, 60, 1, 0) - 1];
    c2_j_y = NULL;
    sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0));
    c2_j_u = 800.0;
    c2_k_y = NULL;
    sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_j_u, 0, 0U, 0U, 0U, 0));
    c2_k_u = 2011.0;
    c2_l_y = NULL;
    sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_k_u, 0, 0U, 0U, 0U, 0));
    sf_mex_assign(&c2_B_ECEF_sph, sf_mex_call_debug("igrf11syn", 1U, 4U, 14,
      c2_l_y, 14, c2_k_y, 14, c2_j_y, 14, c2_i_y));
    _SFD_EML_CALL(0,44);
    sf_mex_assign(&c2_a, sf_mex_dup(c2_B_ECEF_sph));

    /* dip(t)=atand(-a(3)/sqrt(a(2)^2+a(1)^2)); */
    /*          TA(:,:,t)=TargetAttitude([B_ECEF_sph(1);B_ECEF_sph(2);-B_ECEF_sph(3)]); */
    _SFD_EML_CALL(0,47);
    c2_b_B_ECEF_sph = sf_mex_dup(c2_B_ECEF_sph);
    sf_mex_import("B_ECEF_sph", sf_mex_dup(c2_b_B_ECEF_sph), &c2_dv2, 1, 0, 0U,
                  1, 0U, 1, 3);
    for (c2_i13 = 0; c2_i13 < 3; c2_i13 = c2_i13 + 1) {
      c2_TA_at_t[c2_i13] = c2_dv2[c2_i13];
    }

    sf_mex_destroy(&c2_b_B_ECEF_sph);
    c2_b_t = c2_b_t + 1.0;
    sf_mex_listen_for_ctrl_c(chartInstance.S);
  }

  CV_EML_FOR(0, 0, 0);
  _SFD_EML_CALL(0,-47);
  sf_debug_symbol_scope_pop();
  sf_mex_destroy(&c2_r_ECI);
  sf_mex_destroy(&c2_gh);
  sf_mex_destroy(&c2_B_ECEF_sph);
  sf_mex_destroy(&c2_a);
  for (c2_i14 = 0; c2_i14 < 3; c2_i14 = c2_i14 + 1) {
    (*c2_b_TA_at_t)[c2_i14] = c2_TA_at_t[c2_i14];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 0U, sf_debug_get_script_id(
    "C:/IITM Satellite/Codes/Linearized Nominal Mode Control/LatLong.m"));
}

static real_T c2_mrdivide(real_T c2_A, real_T c2_B)
{
  real_T c2_x;
  real_T c2_b_y;
  real_T c2_b_x;
  real_T c2_c_y;
  real_T c2_c_x;
  real_T c2_d_y;
  c2_x = c2_A;
  c2_b_y = c2_B;
  if (c2_b_y == 0.0) {
    c2_eml_warning();
  }

  c2_b_x = c2_x;
  c2_c_y = c2_b_y;
  c2_c_x = c2_b_x;
  c2_d_y = c2_c_y;
  return c2_c_x / c2_d_y;
}

static void c2_eml_warning(void)
{
  int32_T c2_i15;
  static char_T c2_cv0[15] = { 'D', 'i', 'v', 'i', 'd', 'e', ' ', 'b', 'y', ' ',
    'z', 'e', 'r', 'o', '.' };

  char_T c2_u[15];
  const mxArray *c2_y = NULL;
  int32_T c2_i16;
  static char_T c2_cv1[19] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'd', 'i', 'v',
    'i', 'd', 'e', 'B', 'y', 'Z', 'e', 'r', 'o' };

  char_T c2_b_u[19];
  const mxArray *c2_b_y = NULL;
  for (c2_i15 = 0; c2_i15 < 15; c2_i15 = c2_i15 + 1) {
    c2_u[c2_i15] = c2_cv0[c2_i15];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 10, 0U, 1U, 0U, 2, 1, 15));
  for (c2_i16 = 0; c2_i16 < 19; c2_i16 = c2_i16 + 1) {
    c2_b_u[c2_i16] = c2_cv1[c2_i16];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 10, 0U, 1U, 0U, 2, 1, 19));
  sf_mex_call_debug("warning", 0U, 2U, 14, c2_b_y, 14, c2_y);
}

static void c2_LatLong(real_T c2_r_ECEF[3], real_T *c2_lat, real_T *c2_long)
{
  real_T c2_nargout = 2.0;
  real_T c2_nargin = 1.0;
  real_T c2_y;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_b_y;
  real_T c2_d_x;
  real_T c2_c_y;
  real_T c2_e_x;
  real_T c2_d_y;
  real_T c2_f_x;
  sf_debug_symbol_scope_push(5U, 0U);
  sf_debug_symbol_scope_add("nargout", &c2_nargout, c2_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c2_nargin, c2_sf_marshall);
  sf_debug_symbol_scope_add("long", c2_long, c2_sf_marshall);
  sf_debug_symbol_scope_add("lat", c2_lat, c2_sf_marshall);
  sf_debug_symbol_scope_add("r_ECEF", c2_r_ECEF, c2_b_sf_marshall);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0,2);
  c2_y = c2_r_ECEF[2];
  c2_x = c2_mpower(c2_r_ECEF[1]) + c2_mpower(c2_r_ECEF[0]);
  c2_b_x = c2_x;
  if (c2_b_x < 0.0) {
    c2_eml_error();
  }

  c2_c_x = c2_b_x;
  c2_b_x = c2_c_x;
  c2_b_x = muDoubleScalarSqrt(c2_b_x);
  c2_b_y = c2_y;
  c2_d_x = c2_b_x;
  *c2_lat = muDoubleScalarAtan2(c2_b_y, c2_d_x);
  _SFD_SCRIPT_CALL(0,3);
  c2_c_y = c2_r_ECEF[1];
  c2_e_x = c2_r_ECEF[0];
  c2_d_y = c2_c_y;
  c2_f_x = c2_e_x;
  *c2_long = muDoubleScalarAtan2(c2_d_y, c2_f_x);
  _SFD_SCRIPT_CALL(0,-3);
  sf_debug_symbol_scope_pop();
}

static real_T c2_mpower(real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_ak;
  c2_b_a = c2_a;
  c2_ak = c2_b_a;
  return muDoubleScalarPower(c2_ak, 2.0);
}

static void c2_eml_error(void)
{
  int32_T c2_i17;
  static char_T c2_cv2[77] = { 'D', 'o', 'm', 'a', 'i', 'n', ' ', 'e', 'r', 'r',
    'o', 'r', '.', ' ', 'T', 'o', ' ', 'c', 'o', 'm', 'p'
    , 'u', 't', 'e', ' ', 'c', 'o', 'm', 'p', 'l', 'e', 'x', ' ', 'r', 'e', 's',
    'u', 'l', 't', 's', ' ',
    'f', 'r', 'o', 'm', ' ', 'r', 'e', 'a', 'l', ' ', 'x', ',', ' ', 'u', 's',
    'e', ' ', '\'', 's', 'q',
    'r', 't', '(', 'c', 'o', 'm', 'p', 'l', 'e', 'x', '(', 'x', ')', ')', '\'',
    '.' };

  char_T c2_u[77];
  const mxArray *c2_y = NULL;
  int32_T c2_i18;
  static char_T c2_cv3[31] = { 'E', 'm', 'b', 'e', 'd', 'd', 'e', 'd', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'q', 'r', 't', ':', 'd'
    , 'o', 'm', 'a', 'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c2_b_u[31];
  const mxArray *c2_b_y = NULL;
  for (c2_i17 = 0; c2_i17 < 77; c2_i17 = c2_i17 + 1) {
    c2_u[c2_i17] = c2_cv2[c2_i17];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 10, 0U, 1U, 0U, 2, 1, 77));
  for (c2_i18 = 0; c2_i18 < 31; c2_i18 = c2_i18 + 1) {
    c2_b_u[c2_i18] = c2_cv3[c2_i18];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 10, 0U, 1U, 0U, 2, 1, 31));
  sf_mex_call_debug("error", 0U, 2U, 14, c2_b_y, 14, c2_y);
}

static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *((real_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i19;
  real_T c2_b_u[3];
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  for (c2_i19 = 0; c2_i19 < 3; c2_i19 = c2_i19 + 1) {
    c2_b_u[c2_i19] = (*((real_T (*)[3])c2_u))[c2_i19];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 3));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  real_T c2_b_u[540];
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_i20 = 0;
  for (c2_i21 = 0; c2_i21 < 60; c2_i21 = c2_i21 + 1) {
    c2_i22 = 0;
    for (c2_i23 = 0; c2_i23 < 3; c2_i23 = c2_i23 + 1) {
      for (c2_i24 = 0; c2_i24 < 3; c2_i24 = c2_i24 + 1) {
        c2_b_u[(c2_i24 + c2_i22) + c2_i20] = (*((real_T (*)[540])c2_u))[(c2_i24
          + c2_i22) + c2_i20];
      }

      c2_i22 = c2_i22 + 3;
    }

    c2_i20 = c2_i20 + 9;
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 3, 3, 3, 60));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_d_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i25;
  real_T c2_b_u[60];
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  for (c2_i25 = 0; c2_i25 < 60; c2_i25 = c2_i25 + 1) {
    c2_b_u[c2_i25] = (*((real_T (*)[60])c2_u))[c2_i25];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 2, 1, 60));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_e_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  real_T c2_b_u[180];
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_i26 = 0;
  for (c2_i27 = 0; c2_i27 < 60; c2_i27 = c2_i27 + 1) {
    for (c2_i28 = 0; c2_i28 < 3; c2_i28 = c2_i28 + 1) {
      c2_b_u[c2_i28 + c2_i26] = (*((real_T (*)[180])c2_u))[c2_i28 + c2_i26];
    }

    c2_i26 = c2_i26 + 3;
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 2, 3, 60));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_f_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  const mxArray *c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = sf_mex_dup(*((const mxArray **)c2_u));
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_duplicatearraysafe(&c2_b_u));
  sf_mex_destroy(&c2_b_u);
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_TargetAttitudeGenerator_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_ResolvedFunctionInfo c2_info[43];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i29;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 43));
  for (c2_i29 = 0; c2_i29 < 43; c2_i29 = c2_i29 + 1) {
    c2_r0 = &c2_info[c2_i29];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context",
                    "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name",
                    "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c2_i29
                    );
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c2_i29
                    );
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[43])
{
  c2_info[0].context = "";
  c2_info[0].name = "zeros";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[B]zeros";
  c2_info[0].fileLength = 0U;
  c2_info[0].fileTime1 = 0U;
  c2_info[0].fileTime2 = 0U;
  c2_info[1].context = "";
  c2_info[1].name = "pi";
  c2_info[1].dominantType = "";
  c2_info[1].resolved = "[B]pi";
  c2_info[1].fileLength = 0U;
  c2_info[1].fileTime1 = 0U;
  c2_info[1].fileTime2 = 0U;
  c2_info[2].context = "";
  c2_info[2].name = "mtimes";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[2].fileLength = 2408U;
  c2_info[2].fileTime1 = 1227588202U;
  c2_info[2].fileTime2 = 0U;
  c2_info[3].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[3].name = "nargin";
  c2_info[3].dominantType = "";
  c2_info[3].resolved = "[B]nargin";
  c2_info[3].fileLength = 0U;
  c2_info[3].fileTime1 = 0U;
  c2_info[3].fileTime2 = 0U;
  c2_info[4].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[4].name = "gt";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved = "[B]gt";
  c2_info[4].fileLength = 0U;
  c2_info[4].fileTime1 = 0U;
  c2_info[4].fileTime2 = 0U;
  c2_info[5].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[5].name = "isa";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved = "[B]isa";
  c2_info[5].fileLength = 0U;
  c2_info[5].fileTime1 = 0U;
  c2_info[5].fileTime2 = 0U;
  c2_info[6].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[6].name = "isinteger";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved = "[B]isinteger";
  c2_info[6].fileLength = 0U;
  c2_info[6].fileTime1 = 0U;
  c2_info[6].fileTime2 = 0U;
  c2_info[7].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[7].name = "isscalar";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[B]isscalar";
  c2_info[7].fileLength = 0U;
  c2_info[7].fileTime1 = 0U;
  c2_info[7].fileTime2 = 0U;
  c2_info[8].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[8].name = "strcmp";
  c2_info[8].dominantType = "char";
  c2_info[8].resolved = "[B]strcmp";
  c2_info[8].fileLength = 0U;
  c2_info[8].fileTime1 = 0U;
  c2_info[8].fileTime2 = 0U;
  c2_info[9].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[9].name = "size";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved = "[B]size";
  c2_info[9].fileLength = 0U;
  c2_info[9].fileTime1 = 0U;
  c2_info[9].fileTime2 = 0U;
  c2_info[10].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[10].name = "eq";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved = "[B]eq";
  c2_info[10].fileLength = 0U;
  c2_info[10].fileTime1 = 0U;
  c2_info[10].fileTime2 = 0U;
  c2_info[11].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[11].name = "class";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved = "[B]class";
  c2_info[11].fileLength = 0U;
  c2_info[11].fileTime1 = 0U;
  c2_info[11].fileTime2 = 0U;
  c2_info[12].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[12].name = "not";
  c2_info[12].dominantType = "logical";
  c2_info[12].resolved = "[B]not";
  c2_info[12].fileLength = 0U;
  c2_info[12].fileTime1 = 0U;
  c2_info[12].fileTime2 = 0U;
  c2_info[13].context = "";
  c2_info[13].name = "mrdivide";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[13].fileLength = 771U;
  c2_info[13].fileTime1 = 1219731336U;
  c2_info[13].fileTime2 = 0U;
  c2_info[14].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[14].name = "ge";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved = "[B]ge";
  c2_info[14].fileLength = 0U;
  c2_info[14].fileTime1 = 0U;
  c2_info[14].fileTime2 = 0U;
  c2_info[15].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[15].name = "rdivide";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[15].fileLength = 620U;
  c2_info[15].fileTime1 = 1213905166U;
  c2_info[15].fileTime2 = 0U;
  c2_info[16].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[16].name = "isempty";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved = "[B]isempty";
  c2_info[16].fileLength = 0U;
  c2_info[16].fileTime1 = 0U;
  c2_info[16].fileTime2 = 0U;
  c2_info[17].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[17].name = "eml_warning";
  c2_info[17].dominantType = "char";
  c2_info[17].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[17].fileLength = 274U;
  c2_info[17].fileTime1 = 1227588196U;
  c2_info[17].fileTime2 = 0U;
  c2_info[18].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[18].name = "eml_div";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[18].fileLength = 4269U;
  c2_info[18].fileTime1 = 1227588186U;
  c2_info[18].fileTime2 = 0U;
  c2_info[19].context =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_fldiv";
  c2_info[19].name = "isreal";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved = "[B]isreal";
  c2_info[19].fileLength = 0U;
  c2_info[19].fileTime1 = 0U;
  c2_info[19].fileTime2 = 0U;
  c2_info[20].context = "";
  c2_info[20].name = "minus";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved = "[B]minus";
  c2_info[20].fileLength = 0U;
  c2_info[20].fileTime1 = 0U;
  c2_info[20].fileTime2 = 0U;
  c2_info[21].context = "";
  c2_info[21].name = "LatLong";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[]C:/IITM Satellite/Codes/Linearized Nominal Mode Control/LatLong.m";
  c2_info[21].fileLength = 127U;
  c2_info[21].fileTime1 = 1301078010U;
  c2_info[21].fileTime2 = 0U;
  c2_info[22].context =
    "[]C:/IITM Satellite/Codes/Linearized Nominal Mode Control/LatLong.m";
  c2_info[22].name = "mpower";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[22].fileLength = 3623U;
  c2_info[22].fileTime1 = 1227588202U;
  c2_info[22].fileTime2 = 0U;
  c2_info[23].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[23].name = "ndims";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved = "[B]ndims";
  c2_info[23].fileLength = 0U;
  c2_info[23].fileTime1 = 0U;
  c2_info[23].fileTime2 = 0U;
  c2_info[24].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[24].name = "power";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[24].fileLength = 5380U;
  c2_info[24].fileTime1 = 1227588202U;
  c2_info[24].fileTime2 = 0U;
  c2_info[25].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[25].name = "eml_scalar_eg";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[25].fileLength = 3524U;
  c2_info[25].fileTime1 = 1227588194U;
  c2_info[25].fileTime2 = 0U;
  c2_info[26].context =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c2_info[26].name = "false";
  c2_info[26].dominantType = "";
  c2_info[26].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/elmat/false.m";
  c2_info[26].fileLength = 238U;
  c2_info[26].fileTime1 = 1192445120U;
  c2_info[26].fileTime2 = 0U;
  c2_info[27].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/elmat/false.m";
  c2_info[27].name = "logical";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved = "[B]logical";
  c2_info[27].fileLength = 0U;
  c2_info[27].fileTime1 = 0U;
  c2_info[27].fileTime2 = 0U;
  c2_info[28].context =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[28].name = "isstruct";
  c2_info[28].dominantType = "double";
  c2_info[28].resolved = "[B]isstruct";
  c2_info[28].fileLength = 0U;
  c2_info[28].fileTime1 = 0U;
  c2_info[28].fileTime2 = 0U;
  c2_info[29].context =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[29].name = "cast";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/datatypes/cast.m";
  c2_info[29].fileLength = 889U;
  c2_info[29].fileTime1 = 1225964242U;
  c2_info[29].fileTime2 = 0U;
  c2_info[30].context =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/datatypes/cast.m";
  c2_info[30].name = "ischar";
  c2_info[30].dominantType = "char";
  c2_info[30].resolved = "[B]ischar";
  c2_info[30].fileLength = 0U;
  c2_info[30].fileTime1 = 0U;
  c2_info[30].fileTime2 = 0U;
  c2_info[31].context =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/datatypes/cast.m";
  c2_info[31].name = "double";
  c2_info[31].dominantType = "double";
  c2_info[31].resolved = "[B]double";
  c2_info[31].fileLength = 0U;
  c2_info[31].fileTime1 = 0U;
  c2_info[31].fileTime2 = 0U;
  c2_info[32].context =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[32].name = "plus";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved = "[B]plus";
  c2_info[32].fileLength = 0U;
  c2_info[32].fileTime1 = 0U;
  c2_info[32].fileTime2 = 0U;
  c2_info[33].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[33].name = "eml_scalexp_alloc";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[33].fileLength = 794U;
  c2_info[33].fileTime1 = 1227588194U;
  c2_info[33].fileTime2 = 0U;
  c2_info[34].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[34].name = "lt";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved = "[B]lt";
  c2_info[34].fileLength = 0U;
  c2_info[34].fileTime1 = 0U;
  c2_info[34].fileTime2 = 0U;
  c2_info[35].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[35].name = "eml_scalar_floor";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[35].fileLength = 260U;
  c2_info[35].fileTime1 = 1209309190U;
  c2_info[35].fileTime2 = 0U;
  c2_info[36].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[36].name = "ne";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved = "[B]ne";
  c2_info[36].fileLength = 0U;
  c2_info[36].fileTime1 = 0U;
  c2_info[36].fileTime2 = 0U;
  c2_info[37].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[37].name = "eml_error";
  c2_info[37].dominantType = "char";
  c2_info[37].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[37].fileLength = 315U;
  c2_info[37].fileTime1 = 1213905146U;
  c2_info[37].fileTime2 = 0U;
  c2_info[38].context =
    "[]C:/IITM Satellite/Codes/Linearized Nominal Mode Control/LatLong.m";
  c2_info[38].name = "sqrt";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[38].fileLength = 572U;
  c2_info[38].fileTime1 = 1203422846U;
  c2_info[38].fileTime2 = 0U;
  c2_info[39].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[39].name = "eml_scalar_sqrt";
  c2_info[39].dominantType = "double";
  c2_info[39].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[39].fileLength = 664U;
  c2_info[39].fileTime1 = 1209309194U;
  c2_info[39].fileTime2 = 0U;
  c2_info[40].context =
    "[]C:/IITM Satellite/Codes/Linearized Nominal Mode Control/LatLong.m";
  c2_info[40].name = "atan2";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved = "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[40].fileLength = 679U;
  c2_info[40].fileTime1 = 1227588184U;
  c2_info[40].fileTime2 = 0U;
  c2_info[41].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[41].name = "eml_scalexp_compatible";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[41].fileLength = 568U;
  c2_info[41].fileTime1 = 1227588194U;
  c2_info[41].fileTime2 = 0U;
  c2_info[42].context = "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[42].name = "eml_scalar_atan2";
  c2_info[42].dominantType = "double";
  c2_info[42].resolved =
    "[I]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  c2_info[42].fileLength = 964U;
  c2_info[42].fileTime1 = 1209309186U;
  c2_info[42].fileTime2 = 0U;
}

static const mxArray *c2_emlrt_marshallOut(uint8_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 3, 0U, 0U, 0U, 0));
  return c2_y;
}

static void c2_emlrt_marshallIn(const mxArray *c2_TA_at_t, char *c2_name, real_T
  c2_y[3])
{
  real_T c2_dv3[3];
  int32_T c2_i30;
  sf_mex_import(c2_name, sf_mex_dup(c2_TA_at_t), &c2_dv3, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i30 = 0; c2_i30 < 3; c2_i30 = c2_i30 + 1) {
    c2_y[c2_i30] = c2_dv3[c2_i30];
  }

  sf_mex_destroy(&c2_TA_at_t);
}

static uint8_T c2_b_emlrt_marshallIn(const mxArray
  *c2_b_is_active_c2_TargetAttitudeGenerator, char *c2_name)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_name, sf_mex_dup(c2_b_is_active_c2_TargetAttitudeGenerator),
                &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_b_is_active_c2_TargetAttitudeGenerator);
  return c2_y;
}

static void init_io_bus_offset(void)
{
}

static void init_dsm_address_info(void)
{
}

/* SFunction Glue Code */
void sf_c2_TargetAttitudeGenerator_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3625086147U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(533918904U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1286183446U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(263503146U);
}

mxArray *sf_c2_TargetAttitudeGenerator_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(2184854958U);
    pr[1] = (double)(2395482957U);
    pr[2] = (double)(1775721792U);
    pr[3] = (double)(3280153297U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c2_TargetAttitudeGenerator(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"TA_at_t\",},{M[8],M[0],T\"is_active_c2_TargetAttitudeGenerator\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_TargetAttitudeGenerator_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_TargetAttitudeGeneratorMachineNumber_,
           2,
           1,
           1,
           2,
           0,
           0,
           0,
           0,
           1,
           &(chartInstance.chartNumber),
           &(chartInstance.instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_TargetAttitudeGeneratorMachineNumber_,
            chartInstance.chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_TargetAttitudeGeneratorMachineNumber_,chartInstance.chartNumber,1);
          sf_debug_set_chart_event_thresholds
            (_TargetAttitudeGeneratorMachineNumber_,
             chartInstance.chartNumber,
             0,
             0,
             0);
          _SFD_SET_DATA_PROPS(0,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"t",0,
                              c2_sf_marshall);

          {
            unsigned int dimVector[1];
            dimVector[0]= 3;
            _SFD_SET_DATA_PROPS(1,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"TA_at_t",0,c2_b_sf_marshall);
          }

          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of EML Model Coverage */
        _SFD_CV_INIT_EML(0,1,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1517);
        _SFD_CV_INIT_EML_FOR(0,0,844,858,1506);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"LatLong",0,-1,124);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T *c2_t;
          real_T (*c2_TA_at_t)[3];
          c2_t = (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
          c2_TA_at_t = (real_T (*)[3])ssGetOutputPortSignal(chartInstance.S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_t);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_TA_at_t);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_TargetAttitudeGeneratorMachineNumber_,chartInstance.chartNumber,
         chartInstance.instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c2_TargetAttitudeGenerator(void
  *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c2_TargetAttitudeGenerator();
  initialize_c2_TargetAttitudeGenerator();
}

static void sf_opaque_enable_c2_TargetAttitudeGenerator(void *chartInstanceVar)
{
  enable_c2_TargetAttitudeGenerator();
}

static void sf_opaque_disable_c2_TargetAttitudeGenerator(void *chartInstanceVar)
{
  disable_c2_TargetAttitudeGenerator();
}

static void sf_opaque_gateway_c2_TargetAttitudeGenerator(void *chartInstanceVar)
{
  sf_c2_TargetAttitudeGenerator();
}

static mxArray* sf_opaque_get_sim_state_c2_TargetAttitudeGenerator(void
  *chartInstanceVar)
{
  mxArray *st = (mxArray *) get_sim_state_c2_TargetAttitudeGenerator();
  return st;
}

static void sf_opaque_set_sim_state_c2_TargetAttitudeGenerator(void
  *chartInstanceVar, const mxArray *st)
{
  set_sim_state_c2_TargetAttitudeGenerator(sf_mex_dup(st));
}

static void sf_opaque_terminate_c2_TargetAttitudeGenerator(void
  *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S) || sim_mode_is_external
      (chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }

  finalize_c2_TargetAttitudeGenerator();
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_TargetAttitudeGenerator(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_TargetAttitudeGenerator();
  }
}

static void mdlSetWorkWidths_c2_TargetAttitudeGenerator(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("TargetAttitudeGenerator",
      "TargetAttitudeGenerator",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("TargetAttitudeGenerator",
                "TargetAttitudeGenerator",2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("TargetAttitudeGenerator",
      "TargetAttitudeGenerator",2,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"TargetAttitudeGenerator",
        "TargetAttitudeGenerator",2,1);
      sf_mark_chart_reusable_outputs(S,"TargetAttitudeGenerator",
        "TargetAttitudeGenerator",2,1);
    }

    sf_set_rtw_dwork_info(S,"TargetAttitudeGenerator","TargetAttitudeGenerator",
                          2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(929482469U));
  ssSetChecksum1(S,(852438152U));
  ssSetChecksum2(S,(681628622U));
  ssSetChecksum3(S,(1806078055U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_TargetAttitudeGenerator(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "TargetAttitudeGenerator",
      "TargetAttitudeGenerator",2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_TargetAttitudeGenerator(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.initializeChart =
    sf_opaque_initialize_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.terminateChart =
    sf_opaque_terminate_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.enableChart =
    sf_opaque_enable_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.disableChart =
    sf_opaque_disable_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.zeroCrossings = NULL;
  chartInstance.chartInfo.outputs = NULL;
  chartInstance.chartInfo.derivatives = NULL;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.mdlStart = mdlStart_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_TargetAttitudeGenerator;
  chartInstance.chartInfo.extModeExec = NULL;
  chartInstance.chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.storeCurrentConfiguration = NULL;
  chartInstance.S = S;
  ssSetUserData(S,(void *)(&(chartInstance.chartInfo)));/* register the chart instance with simstruct */
  if (!sim_mode_is_rtw_gen(S)) {
    init_dsm_address_info();
  }

  chart_debug_initialization(S,1);
}

void c2_TargetAttitudeGenerator_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_TargetAttitudeGenerator(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_TargetAttitudeGenerator(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_TargetAttitudeGenerator(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_TargetAttitudeGenerator_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
