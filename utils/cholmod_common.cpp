#include "cholmod_common.h"
#include <assert.h>

/******************************************************************************************************************************/
static cholmod_common common;
static bool cholmod_common_initilaized = false;


/******************************************************************************************************************************/
static void cholmod_error_handler(int status, char *file, int line,  char *message)
{
    printf("CHOLMOD error status %d\n", status);
    printf("File: %s\n", file);
    printf("Line: %d\n", line);
    printf("Message: %s\n", message);
}

/******************************************************************************************************************************/
cholmod_common* cholmod_get_common()
{
	assert(cholmod_common_initilaized);
	return &common;
}

/******************************************************************************************************************************/
void cholmod_initialize()
{
	if (!cholmod_common_initilaized) {
			cholmod_common_initilaized = true;
			cholmod_start(&common);
			//common.error_handler = cholmod_error_handler;
			common.final_asis = 1;
			common.supernodal = CHOLMOD_SIMPLICIAL;
	}
}

/******************************************************************************************************************************/
void cholmod_finalize() {
	if (cholmod_common_initilaized)
		cholmod_finish(&common);
}
