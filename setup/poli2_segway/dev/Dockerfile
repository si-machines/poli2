ARG BOILERPLATE_PARENT_IMAGE="osrf/ros"
ARG BOILERPLATE_PARENT_TAG="kinetic-desktop-full"
FROM $BOILERPLATE_PARENT_IMAGE:$BOILERPLATE_PARENT_TAG

MAINTAINER Adam Allevato "<allevato@utexas.edu>"

# Allow docker inside container
RUN apt-get update && \
      apt-get -y install sudo
USER root

COPY setup.sh /setup.sh
RUN /bin/bash /setup.sh


ENTRYPOINT ["entrypoint"]
CMD ["bash"]