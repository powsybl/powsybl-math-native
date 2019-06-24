FROM centos
ARG proxy_host
ARG proxy_port
ARG proxy_username
ARG proxy_password

RUN test -n "${proxy_host}" \
&& echo "proxy=http://${proxy_host}:${proxy_port}" >> /etc/yum.conf \
&& echo "proxy_username=${proxy_username}" >> /etc/yum.conf \
&& echo "proxy_password=${proxy_password}" >> /etc/yum.conf

# epel is needed for cmake3 needed for sparsesuite build 
RUN yum -y install epel-release && \
yum -y update

# epel repo seems not to use same global yum proxy, we need to configure it specifically
RUN test -n "${proxy_host}" \
&& sed -i -e "s/\[epel\]/[epel]\nproxy=http:\/\/${proxy_host}:${proxy_port}\nproxy_username=${proxy_username}\nproxy_password=${proxy_password}/g" /etc/yum.repos.d/epel.repo

RUN yum -y update \
&& yum install -y git cmake3 make gcc-c++ java-1.8.0-openjdk.x86_64 java-1.8.0-openjdk-devel.x86_64 python python-six

ENV http_proxy=http://${proxy_username}:${proxy_password}@${proxy_host}:${proxy_port}
ENV https_proxy=$http_proxy

RUN test -n "${proxy_host}" \
&& echo "[http]" >> $HOME/.gitconfig \
&& echo "	proxy = http://${proxy_username}:${proxy_password}@${proxy_host}:${proxy_port}"  >> $HOME/.gitconfig

# to fix an issue with cmake patch  
RUN git config --global user.email "you@example.com" \
&& git config --global user.name "Your Name"

RUN git clone https://github.com/powsybl/powsybl-math-native.git

RUN mkdir /powsybl-math-native/build \
&& cd /powsybl-math-native/build \
&& cmake3 .. \
&& make
