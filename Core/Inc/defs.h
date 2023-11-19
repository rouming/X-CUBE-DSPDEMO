#ifndef _DEFS_H
#define _DEFS_H

/* Compiler barrier */
#define barrier() __asm__ __volatile__("": : :"memory")

#define __compiletime_warning(message) __attribute__((__warning__(message)))
#define __compiletime_error(message) __attribute__((__error__(message)))

# define __compiletime_assert(condition, msg, prefix, suffix)		\
	do {								\
		extern void prefix ## suffix(void) __compiletime_error(msg); \
		if (!(condition))					\
			prefix ## suffix();				\
	} while (0)

#define _compiletime_assert(condition, msg, prefix, suffix) \
	__compiletime_assert(condition, msg, prefix, suffix)

/**
 * compiletime_assert - break build and emit msg if condition is false
 * @condition: a compile-time constant condition to check
 * @msg:       a message to emit if condition is false
 *
 * In tradition of POSIX assert, this macro will break the build if the
 * supplied condition is *false*, emitting the supplied error message if the
 * compiler has support to do so.
 */
#define compiletime_assert(condition, msg) \
	_compiletime_assert(condition, msg, __compiletime_assert_, __LINE__)


#define BUILD_BUG_ON_ZERO(e) ((int)(sizeof(struct { int:(-!!(e)); })))

/**
 * BUILD_BUG_ON_MSG - break compile if a condition is true & emit supplied
 *		      error message.
 * @condition: the condition which the compiler should know is false.
 *
 * See BUILD_BUG_ON for description.
 */
#define BUILD_BUG_ON_MSG(cond, msg) compiletime_assert(!(cond), msg)

/**
 * BUILD_BUG_ON - break compile if a condition is true.
 * @condition: the condition which the compiler should know is false.
 *
 * If you have some code which relies on certain constants being equal, or
 * some other compile-time-evaluated condition, you should use BUILD_BUG_ON to
 * detect if someone changes it.
 */
#define BUILD_BUG_ON(condition) \
	BUILD_BUG_ON_MSG(condition, "BUILD_BUG_ON failed: " #condition)



/* Are two types/vars the same type (ignoring qualifiers)? */
#define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))

#define __must_be_array(a)	BUILD_BUG_ON_ZERO(__same_type((a), &(a)[0]))

/**
 * ARRAY_SIZE - get the number of elements in array @arr
 * @arr: array to be sized
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]) + __must_be_array(arr))

/* Indirect macros required for expanded argument pasting, eg. __LINE__. */
#define ___PASTE(a,b) a##b
#define __PASTE(a,b) ___PASTE(a,b)

#define __UNIQUE_ID(prefix) __PASTE(__PASTE(__UNIQUE_ID_, prefix), __COUNTER__)

/*
 * min()/max()/clamp() macros must accomplish three things:
 *
 * - avoid multiple evaluations of the arguments (so side-effects like
 *   "x++" happen only once) when non-constant.
 * - perform strict type-checking (to generate warnings instead of
 *   nasty runtime surprises). See the "unnecessary" pointer comparison
 *   in __typecheck().
 * - retain result as a constant expressions when called with only
 *   constant expressions (to avoid tripping VLA warnings in stack
 *   allocation usage).
 */
#define __typecheck(x, y) \
		(!!(sizeof((typeof(x) *)1 == (typeof(y) *)1)))

/*
 * This returns a constant expression while determining if an argument is
 * a constant expression, most importantly without evaluating the argument.
 * Glory to Martin Uecker <Martin.Uecker@med.uni-goettingen.de>
 */
#define __is_constexpr(x) \
	(sizeof(int) == sizeof(*(8 ? ((void *)((long)(x) * 0l)) : (int *)8)))

#define __no_side_effects(x, y) \
		(__is_constexpr(x) && __is_constexpr(y))

#define __safe_cmp(x, y) \
		(__typecheck(x, y) && __no_side_effects(x, y))

#define __cmp(x, y, op)	((x) op (y) ? (x) : (y))

#define __cmp_once(x, y, unique_x, unique_y, op) ({	\
		typeof(x) unique_x = (x);		\
		typeof(y) unique_y = (y);		\
		__cmp(unique_x, unique_y, op); })

#define __careful_cmp(x, y, op) \
	__builtin_choose_expr(__safe_cmp(x, y), \
		__cmp(x, y, op), \
		__cmp_once(x, y, __UNIQUE_ID(__x), __UNIQUE_ID(__y), op))

/**
 * Converts float to millis float
 */
#define float_to_millis(v) ({					\
	(void)__safe_cmp(v, 0.0f);				\
	(v) * 1e3;						\
})

/**
 * Converts float from millis float
 */
#define float_from_millis(v) ({					\
	(void)__safe_cmp(v, 0.0f);				\
	(v) * 1e-3;						\
})

/**
 * MAX - simple variant of max function, can be used in declarations
 */
#define MAX(a,b) ((a) > (b) ? (a) : (b))

/**
 * min - return minimum of two values of the same or compatible types
 * @x: first value
 * @y: second value
 */
#define min(x, y)	__careful_cmp(x, y, <)

/**
 * max - return maximum of two values of the same or compatible types
 * @x: first value
 * @y: second value
 */
#define max(x, y)	__careful_cmp(x, y, >)

/**
 * min_t - return minimum of two values, using the specified type
 * @type: data type to use
 * @x: first value
 * @y: second value
 */
#define min_t(type, x, y)	__careful_cmp((type)(x), (type)(y), <)

/**
 * max_t - return maximum of two values, using the specified type
 * @type: data type to use
 * @x: first value
 * @y: second value
 */
#define max_t(type, x, y)	__careful_cmp((type)(x), (type)(y), >)

/**
 * clamp_t - return a value clamped to a given range using a given type
 * @type: the type of variable to use
 * @val: current value
 * @lo: minimum allowable value
 * @hi: maximum allowable value
 *
 * This macro does no typechecking and uses temporary variables of type
 * @type to make all the comparisons.
 */
#define clamp_t(type, val, lo, hi) min_t(type, max_t(type, val, lo), hi)

/**
 * clamp_val - return a value clamped to a given range using val's type
 * @val: current value
 * @lo: minimum allowable value
 * @hi: maximum allowable value
 *
 * This macro does no typechecking and uses temporary variables of whatever
 * type the input argument @val is.  This is useful when @val is an unsigned
 * type and @lo and @hi are literals that will otherwise be assigned a signed
 * integer type.
 */
#define clamp_val(val, lo, hi) clamp_t(typeof(val), val, lo, hi)

/**
 * swap - swap values of @a and @b
 * @a: first value
 * @b: second value
 */
#define swap(a, b) \
	do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)

#undef offsetof
#ifdef __compiler_offsetof
#define offsetof(TYPE, MEMBER)	__compiler_offsetof(TYPE, MEMBER)
#else
#define offsetof(TYPE, MEMBER)	((size_t)&((TYPE *)0)->MEMBER)
#endif

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({				\
	void *__mptr = (void *)(ptr);					\
	BUILD_BUG_ON_MSG(!__same_type(*(ptr), ((type *)0)->member) &&	\
			 !__same_type(*(ptr), void),			\
			 "pointer type mismatch in container_of()");	\
	((type *)(__mptr - offsetof(type, member))); })

#endif /* _DEFS_H */
