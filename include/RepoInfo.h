#ifndef REPO_INFO_H
#define REPO_INFO_H

// GitHub repository this firmware is released from. The device checks its
// releases for updates. The release workflow sets both from the repository
// it runs in; the defaults below apply to local builds only.
#ifndef TIGOTELL_REPO_OWNER
#define TIGOTELL_REPO_OWNER "gongloo"
#endif
#ifndef TIGOTELL_REPO_NAME
#define TIGOTELL_REPO_NAME "TigoTell"
#endif

#define TIGOTELL_REPO_URL                                                      \
  "https://github.com/" TIGOTELL_REPO_OWNER "/" TIGOTELL_REPO_NAME

#endif // REPO_INFO_H
