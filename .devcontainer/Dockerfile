# syntax=docker/dockerfile:1
FROM mcr.microsoft.com/devcontainers/cpp:0-ubuntu-18.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.27.0-rc3"

COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
    chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

COPY install-dependencies.sh /tmp/

RUN --mount=type=cache,target=/var/cache/apt \
    chmod +x /tmp/install-dependencies.sh && /tmp/install-dependencies.sh

# COPY install-osg.sh /tmp/

# RUN --mount=type=cache,target=/var/cache/apt \
#     chmod +x /tmp/install-osg.sh && /tmp/install-osg.sh

RUN --mount=type=cache,target=/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
    libopenscenegraph-dev

COPY install-fcl.sh /tmp/

RUN --mount=type=cache,target=/var/cache/apt \
    chmod +x /tmp/install-fcl.sh && /tmp/install-fcl.sh

COPY install-openrave.sh /tmp/

RUN --mount=type=cache,target=/var/cache/apt \
    chmod +x /tmp/install-openrave.sh && /tmp/install-openrave.sh

RUN export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
    clang-format-10

USER vscode
CMD ["/bin/zsh"]

