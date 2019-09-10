#
# Copyright (c) 2019, RTE (http://www.rte-france.com)
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#

FROM centos:7.6.1810
MAINTAINER "Mathieu BAGUE <mathieu.bague@rte-france.com>"

# Install additional repositories
RUN yum -y install epel-release

# Install requirements
RUN yum -y install \
    cmake3 \
    gcc-c++ \
    git \
    java-1.8.0-openjdk-devel \
    lapack-devel
    make \
    maven \
    python \
    python-six \
&& yum clean all

# to fix an issue with cmake patch  
RUN git config --global user.email "you@example.com" \
&& git config --global user.name "Your Name"
