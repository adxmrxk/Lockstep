// Generated fixed-arity preprocessor iteration (max 24 fields per message).
// Requires a conformant preprocessor: MSVC needs /Zc:preprocessor (set in CMake).
#define LS_MSG_EXPAND(x) x
#define LS_MSG_CAT_(a, b) a##b
#define LS_MSG_CAT(a, b) LS_MSG_CAT_(a, b)

#define LS_MSG_NARG_(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, N, ...) N
#define LS_MSG_NARG(...) LS_MSG_EXPAND(LS_MSG_NARG_(__VA_ARGS__, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1))

#define LS_MSG_FE_1(m, T, x) m(T, x)
#define LS_MSG_FE_2(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_1(m, T, __VA_ARGS__))
#define LS_MSG_FE_3(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_2(m, T, __VA_ARGS__))
#define LS_MSG_FE_4(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_3(m, T, __VA_ARGS__))
#define LS_MSG_FE_5(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_4(m, T, __VA_ARGS__))
#define LS_MSG_FE_6(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_5(m, T, __VA_ARGS__))
#define LS_MSG_FE_7(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_6(m, T, __VA_ARGS__))
#define LS_MSG_FE_8(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_7(m, T, __VA_ARGS__))
#define LS_MSG_FE_9(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_8(m, T, __VA_ARGS__))
#define LS_MSG_FE_10(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_9(m, T, __VA_ARGS__))
#define LS_MSG_FE_11(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_10(m, T, __VA_ARGS__))
#define LS_MSG_FE_12(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_11(m, T, __VA_ARGS__))
#define LS_MSG_FE_13(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_12(m, T, __VA_ARGS__))
#define LS_MSG_FE_14(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_13(m, T, __VA_ARGS__))
#define LS_MSG_FE_15(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_14(m, T, __VA_ARGS__))
#define LS_MSG_FE_16(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_15(m, T, __VA_ARGS__))
#define LS_MSG_FE_17(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_16(m, T, __VA_ARGS__))
#define LS_MSG_FE_18(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_17(m, T, __VA_ARGS__))
#define LS_MSG_FE_19(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_18(m, T, __VA_ARGS__))
#define LS_MSG_FE_20(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_19(m, T, __VA_ARGS__))
#define LS_MSG_FE_21(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_20(m, T, __VA_ARGS__))
#define LS_MSG_FE_22(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_21(m, T, __VA_ARGS__))
#define LS_MSG_FE_23(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_22(m, T, __VA_ARGS__))
#define LS_MSG_FE_24(m, T, x, ...) m(T, x), LS_MSG_EXPAND(LS_MSG_FE_23(m, T, __VA_ARGS__))

#define LS_MSG_FOR_EACH(m, T, ...) \
  LS_MSG_EXPAND(LS_MSG_CAT(LS_MSG_FE_, LS_MSG_NARG(__VA_ARGS__))(m, T, __VA_ARGS__))
