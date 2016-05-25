#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sc_rv32i__Arch.h"
#include "helper/log.h"
#include "target/target.h"

int error_code__get(struct target const* const p_target)
{
	assert(p_target);
	struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	return p_arch->error_code;
}

int error_code__update(struct target const* const p_target, int const a_error_code)
{
	assert(p_target);
	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	if ( ERROR_OK == error_code__get(p_target) && ERROR_OK != a_error_code ) {
		LOG_DEBUG("Set new error code: %d", a_error_code);
		p_arch->error_code = a_error_code;
	}
	return error_code__get(p_target);
}

int error_code__get_and_clear(struct target const* const p_target)
{
	assert(p_target);
	struct sc_rv32i__Arch* const const p_arch = p_target->arch_info;
	assert(p_arch);
	int const result = error_code__get(p_target);
	p_arch->error_code = ERROR_OK;
	return result;
}

int error_code__prepend(struct target const* const p_target, int const old_err_code)
{
	assert(p_target);
	int const new_err_code = error_code__get_and_clear(p_target);
	error_code__update(p_target, old_err_code);
	error_code__update(p_target, new_err_code);
	return error_code__get(p_target);
}
