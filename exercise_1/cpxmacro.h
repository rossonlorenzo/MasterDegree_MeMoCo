/**
 * @file cpx_macro.h
 * @brief Modernized CPLEX Helper Macros without global variables
 */

#ifndef CPX_MACRO_H
#define CPX_MACRO_H

#include <cstring>
#include <string>
#include <stdexcept>
#include <ilcplex/cplex.h>

#define STRINGIZE(something) STRINGIZE_HELPER(something)
#define STRINGIZE_HELPER(something) #something

/**
 * typedefs for basic Callable Library entities
 */
typedef CPXENVptr Env;
typedef CPXLPptr Prob;

/**
 * Shortcut for declaring a CPLEX environment with error checking
 */
#define DECL_ENV(name) \
Env name = CPXopenCPLEX(&status); \
{ \
    int local_status = status; \
    if (local_status) { \
        char errmsg_buf[4096]; \
        CPXgeterrorstring(NULL, local_status, errmsg_buf); \
        int trailer = std::strlen(errmsg_buf) - 1; \
        if (trailer >= 0) errmsg_buf[trailer] = '\0'; \
        throw std::runtime_error(std::string(__FILE__) + ":" + STRINGIZE(__LINE__) + ": " + errmsg_buf); \
    } \
}

/**
 * Shortcut for declaring a CPLEX problem with error checking
 */
#define DECL_PROB(env, name) \
Prob name = CPXcreateprob(env, &status, ""); \
{ \
    int local_status = status; \
    if (local_status) { \
        char errmsg_buf[4096]; \
        CPXgeterrorstring(env, local_status, errmsg_buf); \
        int trailer = std::strlen(errmsg_buf) - 1; \
        if (trailer >= 0) errmsg_buf[trailer] = '\0'; \
        throw std::runtime_error(std::string(__FILE__) + ":" + STRINGIZE(__LINE__) + ": " + errmsg_buf); \
    } \
}

/**
 * Make a checked call to any CPLEX API function
 * Usage: CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &obj, &lb, &ub, &ctype, &xname);
 */
#define CHECKED_CPX_CALL(func, env, ...) do { \
    int local_status = func(env, __VA_ARGS__); \
    if (local_status) { \
        char errmsg_buf[4096]; \
        CPXgeterrorstring(env, local_status, errmsg_buf); \
        int trailer = std::strlen(errmsg_buf) - 1; \
        if (trailer >= 0) errmsg_buf[trailer] = '\0'; \
        throw std::runtime_error(std::string(__FILE__) + ":" + STRINGIZE(__LINE__) + ": " + errmsg_buf); \
    } \
} while(false)

#endif /* CPX_MACRO_H */
