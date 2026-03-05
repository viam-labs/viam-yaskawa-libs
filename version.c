#include "version.h"

#define COMMS_VERSION_STRING "1.0.0"

#ifndef COMMS_GIT_COMMIT_SHORT
#define COMMS_GIT_COMMIT_SHORT "unknown"
#endif

const char *comms_version_string(void) {
    return COMMS_VERSION_STRING;
}
const char *comms_git_commit(void) {
    return COMMS_GIT_COMMIT_SHORT;
}
