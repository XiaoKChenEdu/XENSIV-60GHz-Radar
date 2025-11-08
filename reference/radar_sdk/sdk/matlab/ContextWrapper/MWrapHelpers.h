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

#ifndef MWRAP_HELPERS_H
#define MWRAP_HELPERS_H

#include <ifxBase/Types.h> // for ifx_Float_t

#include "mex.h"

#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef mxArray xArray;

typedef struct WrapperContext_s WrapperContext;

typedef void (*WrapFn)(WrapperContext *);

typedef struct {
    const char *name;
    WrapFn fn;
    int required_args_left;
    int required_args_right;
} CommandDescriptor;

typedef struct {
    const char *api_name;
    const CommandDescriptor *commands;
} Wrapper;

extern const Wrapper wrapper;
extern const CommandDescriptor internal_commands[];

/**
 * \defgroup arg_x group of functions
 * @brief Return the type specific input parameter specified by the argnum argument number.
 * @param ctx the mex callable context wrapper.
 * @param argnum the context wrapper input argument number.
 * @retval the specific type.
 * @{
 */
extern const xArray *arg(const WrapperContext *ctx, int argnum);
extern const xArray *arg_class_x(const WrapperContext *ctx, int argnum, const char *required_class);
extern char *arg_string(const WrapperContext *ctx, int argnum);
extern bool arg_bool(const WrapperContext* ctx, int argnum);
extern ifx_Float_t arg_float(const WrapperContext* ctx, int argnum);
extern double arg_double(const WrapperContext* ctx, int argnum);
extern uint64_t arg_uint64(const WrapperContext *ctx, int argnum);
extern uint32_t arg_uint32(const WrapperContext *ctx, int argnum);
extern uint16_t arg_uint16(const WrapperContext *ctx, int argnum);
extern uint8_t arg_uint8(const WrapperContext *ctx, int argnum);
extern size_t arg_sizet(const WrapperContext *ctx, int argnum);
extern void *arg_pointer(const WrapperContext *ctx, int argnum);
extern void *arg_pointer_valid(const WrapperContext *ctx, int argnum);
/** @} */

/**
 * \defgroup ret_x group of functions
 * @brief Set the specific output parameter specified by the argnum argument number.
 * @param ctx the mex callable context wrapper.
 * @param argnum the context wrapper output argument number.
 * @{
 */
extern void ret(WrapperContext *ctx, int argnum, xArray *value);
extern void ret_error(WrapperContext *ctx, int argnum);
extern void ret_bool(WrapperContext *ctx, int argnum, bool v);
extern void ret_float(WrapperContext* ctx, int argnum, ifx_Float_t v);
extern void ret_double(WrapperContext* ctx, int argnum, double v);
extern void ret_uint8(WrapperContext *ctx, int argnum, uint8_t v);
extern void ret_uint32(WrapperContext *ctx, int argnum, uint32_t v);
extern void ret_uint64(WrapperContext *ctx, int argnum, uint64_t v);
extern void ret_sizet(WrapperContext *ctx, int argnum, size_t v);
extern void ret_pointer(WrapperContext *ctx, int argnum, void *v);
extern void ret_string(WrapperContext *ctx, int argnum, const char *v);
/** @} */

/**
 * \defgroup pset_x group of functions
 * @brief Set the specific mxArray attribute.
 * @param xArray the mxArray which attribute is to be set.
 * @param idx the offset from which the attribute is being set.
 * @param name the name of the attribute to be set.
 * @{
 */
extern void pset_array(xArray *a, size_t idx, const char *name, const xArray *v);
extern void pset_string(xArray *a, size_t idx, const char *name, const char *v);
extern void pset_uint16(xArray *a, size_t idx, const char *name, uint16_t v);
extern void pset_uint32(xArray *a, size_t idx, const char *name, uint32_t v);
extern void pset_uint64(xArray *a, size_t idx, const char *name, uint64_t v);
extern void pset_uint8(xArray *a, size_t idx, const char *name, uint8_t v);
extern void pset_float(xArray *a, size_t idx, const char *name, float v);
extern void pset_double(xArray *a, size_t idx, const char *name, double v);
extern void pset_bool(xArray* a, size_t idx, const char* name, bool v);
/** @} */

/**
 * \defgroup pget_x group of functions
 * @brief Get the specific mxArray attribute.
 * @param xArray the mxArray which attribute is to be retrieved from.
 * @param idx the offset from which the attribute is being retrieved.
 * @param name the name of the attribute to be retrieved.
 * @{
 */
extern uint32_t pget_uint32(const xArray *a, size_t idx, const char *name);
extern uint64_t pget_uint64(const xArray *a, size_t idx, const char *name);
extern uint8_t pget_uint8(const xArray *a, size_t idx, const char *name);
extern uint16_t pget_uint16(const xArray* a, size_t idx, const char* name);
extern bool pget_bool(const xArray* a, size_t idx, const char* name);
extern float pget_float(const xArray *a, size_t idx, const char *name);
extern double pget_double(const xArray *a, size_t idx, const char *name);
/** @} */

#ifdef __cplusplus
} // extern "C"
#endif

#endif
