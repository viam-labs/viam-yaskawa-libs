#include "version.h"
#include <string.h>

#define COMMS_VERSION_STRING "1.0.0"

#ifndef COMMS_GIT_COMMIT_SHORT
#define COMMS_GIT_COMMIT_SHORT "unknown"
#endif

void comms_get_version_info(version_info_payload_t *info) {
    memset(info, 0, sizeof(*info));
    info->protocol_version = PROTOCOL_VERSION;
    strncpy(info->version_string, COMMS_VERSION_STRING, sizeof(info->version_string) - 1);
    strncpy(info->git_commit, COMMS_GIT_COMMIT_SHORT, sizeof(info->git_commit) - 1);
}

const char *comms_version_string(void) {
    return COMMS_VERSION_STRING;
}
const char *comms_git_commit(void) {
    return COMMS_GIT_COMMIT_SHORT;
}
