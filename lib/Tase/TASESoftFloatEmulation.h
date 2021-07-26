//Header file for soft float  functions.  The idea is that we can
//intercept these function calls in TASE, and concretize the operands before the
//functions are called.


//See https://gcc.gnu.org/onlinedocs/gccint/Soft-float-library-routines.html
//for more information

#ifndef SF_IMPL_H
#define SF_IMPL_H

#ifdef __cplusplus
extern "C" {
#endif

  //First, the tase model versions of the functions that we trap on:

  //Arithmetic
  float  __addsf3_tase(float x, float y);
  double __adddf3_tase(double x, double y);

  float  __subsf3_tase(float x, float y);
  double __subdf3_tase(double x, double y);

  float  __mulsf3_tase(float x, float y);
  double __muldf3_tase(double x, double y);

  float  __divsf3_tase(float x, float y);
  double __divdf3_tase(double x, double y);

  float  __negsf2_tase(float x);
  double __negdf2_tase(double x);

  //Conversion
  double __extendsfdf2_tase(float x);

  float  __truncdfsf2_tase(double x);

  int    __fixsfsi_tase (float  x);
  int    __fixdfsi_tase (double x);

  long   __fixsfdi_tase (float  x);
  long   __fixdfdi_tase (double x);

  long long __fixsfti_tase (float  x);
  long long __fixdfti_tase (double x);

  unsigned int __fixunssfsi_tase (float x);
  unsigned int __fixunsdfsi_tase (double x);

  unsigned long __fixunssfdi_tase (float x);
  unsigned long __fixunsdfdi_tase (double x);

  unsigned long long __fixunssfti_tase (float x);
  unsigned long long __fixunsdfti_tase (double x);

  float  __floatsisf_tase (int x);
  double __floatsidf_tase (int x);

  float  __floatdisf_tase (long x);
  double __floatdidf_tase (long x);

  float  __floattisf_tase (long long x);
  double __floattidf_tase (long long x);

  float  __floatunsisf_tase (unsigned int x);
  double __floatunsidf_tase (unsigned int x);

  float  __floatundisf_tase (unsigned int x);
  double __floatundidf_tase (unsigned int x);

  float  __floatuntisf_tase  (unsigned long long x);
  double __floatuntidf_tase  (unsigned long long x);

  //Comparison

  int __cmpsf2_tase (float x, float y);
  int __cmpdf2_tase (double x, double y);

  int __unordsf2_tase (float x, float y);
  int __unorddf2_tase (double x, double y);

  int __eqsf2_tase (float x, float y);
  int __eqdf2_tase (double x, double y);

  int __nesf2_tase (float x, float y);
  int __nedf2_tase (double x, double y);

  int __gesf2_tase (float x, float y);
  int __gedf2_tase (double x, double y);

  int __ltsf2_tase (float x, float y);
  int __ltdf2_tase (double x, double y);

  int __lesf2_tase (float x, float y);
  int __ledf2_tase (double x, double y);

  int __gtsf2_tase (float x, float y);
  int __gtdf2_tase (double x, double y);

  //Other

  float  __powisf2_tase (float x,  int y);
  double __powidf2_tase (double x, int y);
  

  //Actual fns from the gcc soft float library:
//Arithmetic functions

 float __addsf3 (float a, float b);
 double __adddf3 (double a, double b);
 long double __addtf3 (long double a, long double b);
 long double __addxf3 (long double a, long double b);

 float __subsf3 (float a, float b);
 double __subdf3 (double a, double b);
 long double __subtf3 (long double a, long double b);
 long double __subxf3 (long double a, long double b);

 float __mulsf3 (float a, float b);
 double __muldf3 (double a, double b);
 long double __multf3 (long double a, long double b);
 long double __mulxf3 (long double a, long double b);

 float __divsf3 (float a, float b);
 double __divdf3 (double a, double b);
 long double __divtf3 (long double a, long double b);
 long double __divxf3 (long double a, long double b);

 float __negsf2 (float a);
 double __negdf2 (double a);
 long double __negtf2 (long double a);
 long double __negxf2 (long double a);

//Conversion functions

 double __extendsfdf2 (float a);
 long double __extendsftf2 (float a);
 long double __extendsfxf2 (float a);
 long double __extenddftf2 (double a);
 long double __extenddfxf2 (double a);

 double __truncxfdf2 (long double a);
 double __trunctfdf2 (long double a);
 float __truncxfsf2 (long double a);
 float __trunctfsf2 (long double a);
 float __truncdfsf2 (double a);

 int __fixsfsi (float a);
 int __fixdfsi (double a);
 int __fixtfsi (long double a);
 int __fixxfsi (long double a);

 long __fixsfdi (float a);
 long __fixdfdi (double a);
 long __fixtfdi (long double a);
 long __fixxfdi (long double a);

 long long __fixsfti (float a);
 long long __fixdfti (double a);
 long long __fixtfti (long double a);
 long long __fixxfti (long double a);

 unsigned int __fixunssfsi (float a);
 unsigned int __fixunsdfsi (double a);
 unsigned int __fixunstfsi (long double a);
 unsigned int __fixunsxfsi (long double a);

 unsigned long __fixunssfdi (float a);
 unsigned long __fixunsdfdi (double a);
 unsigned long __fixunstfdi (long double a);
 unsigned long __fixunsxfdi (long double a);

 unsigned long long __fixunssfti (float a);
 unsigned long long __fixunsdfti (double a);
 unsigned long long __fixunstfti (long double a);
 unsigned long long __fixunsxfti (long double a);

 float __floatsisf (int i);
 double __floatsidf (int i);
 long double __floatsitf (int i);
 long double __floatsixf (int i);

 float __floatdisf (long i);
 double __floatdidf (long i);
 long double __floatditf (long i);
 long double __floatdixf (long i);

 float __floattisf (long long i);
 double __floattidf (long long i);
 long double __floattitf (long long i);
 long double __floattixf (long long i);

 float __floatunsisf (unsigned int i);
 double __floatunsidf (unsigned int i);
 long double __floatunsitf (unsigned int i);
 long double __floatunsixf (unsigned int i);

 float __floatundisf (unsigned long i);
 double __floatundidf (unsigned long i);
 long double __floatunditf (unsigned long i);
 long double __floatundixf (unsigned long i);

 float __floatuntisf (unsigned long long i);
 double __floatuntidf (unsigned long long i);
 long double __floatuntitf (unsigned long long i);
 long double __floatuntixf (unsigned long long i);

//Comparison functions

 int __cmpsf2 (float a, float b);
 int __cmpdf2 (double a, double b);
 int __cmptf2 (long double a, long double b);

 int __unordsf2 (float a, float b);
 int __unorddf2 (double a, double b);
 int __unordtf2 (long double a, long double b);

 int __eqsf2 (float a, float b);
 int __eqdf2 (double a, double b);
 int __eqtf2 (long double a, long double b);

 int __nesf2 (float a, float b);
 int __nedf2 (double a, double b);
 int __netf2 (long double a, long double b);

 int __gesf2 (float a, float b);
 int __gedf2 (double a, double b);
 int __getf2 (long double a, long double b);

 int __ltsf2 (float a, float b);
 int __ltdf2 (double a, double b);
 int __lttf2 (long double a, long double b);

 int __lesf2 (float a, float b);
 int __ledf2 (double a, double b);
 int __letf2 (long double a, long double b);

 int __gtsf2 (float a, float b);
 int __gtdf2 (double a, double b);
 int __gttf2 (long double a, long double b);

//Other functions

 float __powisf2 (float a, int b);
 double __powidf2 (double a, int b);
 long double __powitf2 (long double a, int b);
 long double __powixf2 (long double a, int b);

#ifdef __cplusplus
}
#endif

//For now at least, ignore the complex soft float functions
/*
 complex float __mulsc3 (float a, float b, float c, float d);
 complex double __muldc3 (double a, double b, double c, double d);
 complex long double __multc3 (long double a, long double b, long double c, long double d);
 complex long double __mulxc3 (long double a, long double b, long double c, long double d);

 complex float __divsc3 (float a, float b, float c, float d);
 complex double __divdc3 (double a, double b, double c, double d);
 complex long double __divtc3 (long double a, long double b, long double c, long double d);
 complex long double __divxc3 (long double a, long double b, long double c, long double d);
*/
#endif
