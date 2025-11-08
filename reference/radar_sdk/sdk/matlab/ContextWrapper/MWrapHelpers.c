/* ===========================================================================
** Copyright (C) 2021 Infineon Technologies AG
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
** INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
** CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
** ===========================================================================
*/

#define MWRAP_HELPERS_USE_MEX
#include "../ContextWrapper/MWrapHelpers.h"
#include "ifxBase/Error.h"

#ifndef mxIsString
#   define mxIsString mxIsChar
#endif

struct WrapperContext_s {
    const xArray **args_in;
    xArray **args_out;
    int len_args_in;
    int len_args_out;
    const char *func_name;
};

/* The gateway function */
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    if(nrhs < 1)
    {
        mexErrMsgIdAndTxt("sdk:function_not_specified",
                          "function name needs to be specified.");
    }

    if(!mxIsString(prhs[0]))
    {
        mexErrMsgIdAndTxt("sdk:function_must_be_string",
                          "function name should be a string.");
    }

    char* func_name = mxArrayToString (prhs[0]);
    bool is_internal_command = (func_name[0] == ':');
    const CommandDescriptor *commands = 
        is_internal_command ? internal_commands : wrapper.commands;

    bool found = false;
    for(size_t i = 0; commands[i].fn != NULL; i++)
    {
        const CommandDescriptor *cmd = &commands[i];
        if(strcmp(func_name, cmd->name) == 0)
        {
            found = true;
            if(cmd->required_args_left != nlhs)
            {
                mexErrMsgIdAndTxt("sdk:wrapper:missinglhs",
                                  "missing arguments on left hand side of function.");
            }
            else if(cmd->required_args_right + 1 != nrhs)
            {
                mexErrMsgIdAndTxt("sdk:wrapper:missingrhs",
                                  "missing arguments on right hand side of function.");
            }
            else
            {
                WrapperContext ctx = {
                    prhs + 1, // skip first argument
                    plhs,
                    nrhs - 1,
                    nlhs,
                    func_name
                };
                /* call found function, but skip the first argument on the rhs */
                (cmd->fn)(&ctx);
            }
            

            break;
        }
    }
    if(!found)
    {
        mexErrMsgIdAndTxt("sdk:wrapper:unknown_function",
                          "specified device control function not supported.");
    }

    mxFree(func_name);
}

static void errMsgIdAndTxt(const WrapperContext *ctx, const char *description)
{
    size_t an_len = strlen(wrapper.api_name);
    size_t fn_len = strlen(ctx->func_name);
    char *group = mxMalloc(an_len + fn_len + 2);
    memcpy(group, wrapper.api_name, an_len);
    group[an_len] = ':';
    memcpy(group + an_len + 1, ctx->func_name, fn_len);
    group[an_len + 1 + fn_len] = '\0';

    mexErrMsgIdAndTxt(group, description);
}

xArray *pack_bool(bool dat)
{
    mxArray *ret = mxCreateLogicalScalar((char)dat);
    return ret;
}

xArray* pack_float(float dat)
{
    mxArray* ret = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* dh = (float*)mxGetData(ret);
    *dh = dat;
    return ret;
}

xArray* pack_double(double dat)
{
    mxArray* ret = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double* dh = (double*)mxGetData(ret);
    *dh = dat;
    return ret;
}

xArray *pack_uint16(uint16_t dat)
{
    mxArray *ret = mxCreateNumericMatrix(1, 1, mxUINT16_CLASS, mxREAL);
    uint16_t* dh = (uint16_t*) mxGetData(ret);
    *dh = dat;
    return ret;
}

xArray *pack_uint32(uint32_t dat)
{
    mxArray *ret = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    uint32_t* dh = (uint32_t*) mxGetData(ret);
    *dh = dat;
    return ret;
}

xArray *pack_uint64(uint64_t dat)
{
    mxArray *ret = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    uint64_t* dh = (uint64_t*) mxGetData(ret);
    *dh = dat;
    return ret;
}

xArray *pack_uint8(uint8_t dat)
{
    mxArray *ret = mxCreateNumericMatrix(1, 1, mxUINT8_CLASS, mxREAL);
    uint8_t* dh = (uint8_t*) mxGetData(ret);
    *dh = dat;
    return ret;
}

xArray *pack_sizet(size_t dat)
{
    return pack_uint64((uint64_t)dat);
}

xArray *pack_pointer(void *ptr)
{
    return pack_uint64((uint64_t)ptr);
}

xArray *pack_string(const char *dat)
{
    mxArray *ret = mxCreateString(dat);
    return ret;
}

const xArray *arg(const WrapperContext *ctx, int argnum)
{
    if((argnum < 0) || (argnum >= ctx->len_args_in))
    {
        errMsgIdAndTxt(ctx, "input argument out of bounds");
    }
    return ctx->args_in[argnum];
}

const xArray *arg_class_x(const WrapperContext *ctx, int argnum, const char *required_class)
{
    const xArray *a = arg(ctx, argnum);

    if(!mxIsClass(a, required_class)) {
        // FIXME, here we could generate a better error message including the arg number and required class name
        errMsgIdAndTxt(ctx, "input argument must be proper class");
    }

    return a;
}

char *arg_string(const WrapperContext *ctx, int argnum)
{
    const xArray *a = arg(ctx, argnum);

    if(!mxIsString(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be string");
    }

    return mxArrayToString(a);
}

bool arg_bool(const WrapperContext* ctx, int argnum)
{
    const xArray* a = arg(ctx, argnum);

    if (!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }

    return (bool)mxGetScalar(a);
}

float arg_float(const WrapperContext* ctx, int argnum)
{
    const xArray* a = arg(ctx, argnum);

    if (!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }

    return (float)mxGetScalar(a);
}

double arg_double(const WrapperContext* ctx, int argnum)
{
    const xArray* a = arg(ctx, argnum);

    if (!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }

    return (double)mxGetScalar(a);
}

uint64_t arg_uint64(const WrapperContext *ctx, int argnum)
{
    const xArray *a = arg(ctx, argnum);

    if(!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }

    return (uint64_t)mxGetScalar(a);
}

uint32_t arg_uint32(const WrapperContext *ctx, int argnum)
{
    const xArray *a = arg(ctx, argnum);

    if(!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }

    return (uint32_t)mxGetScalar(a);
}

uint16_t arg_uint16(const WrapperContext *ctx, int argnum)
{
    const xArray *a = arg(ctx, argnum);

    if(!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }

    return (uint16_t)mxGetScalar(a);
}

uint8_t arg_uint8(const WrapperContext *ctx, int argnum)
{
    const xArray *a = arg(ctx, argnum);

    if(!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }

    return (uint8_t)mxGetScalar(a);
}

size_t arg_sizet(const WrapperContext *ctx, int argnum)
{
    const xArray *a = arg(ctx, argnum);

    if(!mxIsScalar(a) || !mxIsNumeric(a)) {
        // FIXME, here we could generate a better error message including the arg number
        errMsgIdAndTxt(ctx, "input argument must be scalar number");
    }
    /* FIXME: we might need some other logic here for integers which don't fit into a double.
     *        Some helper which matches on the class of a might do the trick */ 
    return (size_t)mxGetScalar(a);
}

void *arg_pointer(const WrapperContext *ctx, int argnum)
{
    const xArray *a = arg(ctx, argnum);

    // FIXME check that the structure in the a pointer corresponds 
    // to what ret_pointer returns
    uint64_t* dh = (uint64_t*)mxGetData(a);
    void *ptr = (void *)(*dh);

    return ptr;
}

void *arg_pointer_valid(const WrapperContext *ctx, int argnum)
{
    void *ptr = arg_pointer(ctx, argnum);

    if(ptr == NULL)
    {
        errMsgIdAndTxt(ctx, "pointer is null");
    }

    return ptr;
}

void ret(WrapperContext *ctx, int argnum, xArray *value)
{
    if((argnum < 0) || (argnum >= ctx->len_args_out))
    {
        errMsgIdAndTxt(ctx, "return value index out of bounds");
    }

    ctx->args_out[argnum] = value;
}

void ret_error(WrapperContext *ctx, int argnum)
{
    ifx_Error_t err_code = ifx_error_get_and_clear();
    ret_uint32(ctx, argnum, (uint32_t)err_code);
}

void ret_bool(WrapperContext *ctx, int argnum, bool v)
{
    ret(ctx, argnum, pack_bool(v));
}

void ret_float(WrapperContext* ctx, int argnum, float v)
{
    ret(ctx, argnum, pack_float(v));
}

void ret_double(WrapperContext* ctx, int argnum, double v)
{
    ret(ctx, argnum, pack_double(v));
}

void ret_uint8(WrapperContext *ctx, int argnum, uint8_t v)
{
    ret(ctx, argnum, pack_uint8(v));
}

void ret_uint32(WrapperContext *ctx, int argnum, uint32_t v)
{
    ret(ctx, argnum, pack_uint32(v));
}

void ret_uint64(WrapperContext *ctx, int argnum, uint64_t v)
{
    ret(ctx, argnum, pack_uint64(v));
}

void ret_sizet(WrapperContext *ctx, int argnum, size_t v)
{
    ret(ctx, argnum, pack_sizet(v));
}

void ret_pointer(WrapperContext *ctx, int argnum, void *v)
{
    ret(ctx, argnum, pack_pointer(v));
}

void ret_string(WrapperContext *ctx, int argnum, const char *v)
{
    ret(ctx, argnum, pack_string(v));
}

void pset_array(xArray *a, size_t idx, const char *name, const xArray *v)
{
    mxSetProperty(a, (mwIndex)idx, name, v);
}

void pset_string(xArray *a, size_t idx, const char *name, const char *v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_string(v));
}

void pset_uint16(xArray *a, size_t idx, const char *name, uint16_t v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_uint16(v));
}

void pset_uint32(xArray *a, size_t idx, const char *name, uint32_t v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_uint32(v));
}

void pset_uint64(xArray *a, size_t idx, const char *name, uint64_t v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_uint64(v));
}

void pset_uint8(xArray *a, size_t idx, const char *name, uint8_t v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_uint8(v));
}

void pset_float(xArray *a, size_t idx, const char *name, float v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_float(v));
}

void pset_double(xArray *a, size_t idx, const char *name, double v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_double(v));
}

void pset_bool(xArray* a, size_t idx, const char* name, bool v)
{
    mxSetProperty(a, (mwIndex)idx, name, pack_bool(v));
}

uint32_t pget_uint32(const xArray *a, size_t idx, const char *name)
{
    return (uint32_t)mxGetScalar(mxGetProperty(a, (mwIndex)idx, name));
}

uint64_t pget_uint64(const xArray *a, size_t idx, const char *name)
{
    return (uint64_t)mxGetScalar(mxGetProperty(a, (mwIndex)idx, name));
}

uint8_t pget_uint8(const xArray *a, size_t idx, const char *name)
{
    return (uint8_t)mxGetScalar(mxGetProperty(a, (mwIndex)idx, name));
}

uint16_t pget_uint16(const xArray* a, size_t idx, const char* name)
{
    return (uint16_t)mxGetScalar(mxGetProperty(a, (mwIndex)idx, name));
}

bool pget_bool(const xArray* a, size_t idx, const char* name)
{
    return (bool)mxGetScalar(mxGetProperty(a, (mwIndex)idx, name));
}

float pget_float(const xArray *a, size_t idx, const char *name)
{
    return (float)mxGetScalar(mxGetProperty(a, (mwIndex)idx, name));
}

double pget_double(const xArray *a, size_t idx, const char *name)
{
    return (double)mxGetScalar(mxGetProperty(a, (mwIndex)idx, name));
}
