# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #
#  Makefile method of doing things should you prefer
#
# Configure shell
SHELL = bash -eu -o pipefail

# Variables
THIS_MAKEFILE	:= $(abspath $(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))
WORKING_DIR		:= $(dir $(THIS_MAKEFILE))
PACKAGE_DIR     := $(WORKING_DIR)robot
TEST_DIR        := $(WORKING_DIR)tests

include .make/setup.mk

# Variables

VENVDIR         := .venv
TESTVENVDIR		:= $(VENVDIR)-test
PYVERSION       ?= ${PYVERSION:-"3.14"}
PYTHON          := python${PYVERSION}
REQUIREMENTS    ?= requirements.txt

COVERAGE_OPTS	 = --with-xcoverage --with-xunit \
                   --cover-html --cover-html-dir=tmp/cover

VERSION         := $(shell grep VERSION ${PACKAGE_DIR}/version.py  | awk '{ print $$3 }' | sed 's/"//g')

# OpenTelemetry Options
OTEL_SERVICE_NAME 	  := cyberjagzz
OTEL_TRACE_EXPORTER	  := console,otlp
OTEL_METRICS_EXPORTER := console
OTEL_OLTP_ENDPOINT    := supermicro:4317

# Lint tools
PYLINT_DISABLES  = -d similarities -d broad-except -d missing-class-docstring
PYLINT_OPTS		 = -j 4 --exit-zero --rcfile=${WORKING_DIR}.pylintrc $(PYLINT_DISABLES)
PYLINT_OUT		 = $(WORKING_DIR)pylint.out

LICENSE_OUT      = $(WORKING_DIR)license-check.out

.PHONY: venv venv-test venv-sudo test clean distclean deploy sync

## Defaults
default: help		## Default operation is to print this help text

# TODO: OpenTelemetry is a wish list item for development use
# include .make/telemetry.mk        # OpenTelemetry support

## Installation and project maintenance commands
deploy:		## Install/deploy to the application on the roboRIO
	(cd ${PACKAGE_DIR} && python3 -m robotpy deploy)

sync:		## Synchronize this project with the pyproject.toml
	(cd ${PACKAGE_DIR} && python3 -m robotpy sync)

## Virtual Environment
venv: ${PACKAGE_DIR}/$(REQUIREMENTS) $(VENVDIR)/.built		    ## Application virtual environment

venv-test: $(TEST_DIR)/requirements.txt $(TESTVENVDIR)/.built   ## Unit-test/lint/... virtual environment

$(VENVDIR)/.built:
	$(Q) (cd ${PACKAGE_DIR} 				 && \
          ${PYTHON} -m venv ${VENVDIR}		 && \
          chmod a+x ${VENVDIR}/bin/activate  && \
          (source ${VENVDIR}/bin/activate    && \
          if python -m pip install --disable-pip-version-check -r $(REQUIREMENTS); then \
              uname -s > ${VENVDIR}/.built; \
          fi))

$(TESTVENVDIR)/.built:
	$(Q) (cd ${PACKAGE_DIR} 				     && \
          ${PYTHON} -m venv ${TESTVENVDIR}		 && \
          chmod a+x ${TESTVENVDIR}/bin/activate  && \
          (source ${TESTVENVDIR}/bin/activate    && \
          if python -m pip install --disable-pip-version-check -r tests/requirements.txt; \
          then \
	          python -m pip install --disable-pip-version-check pylint; \
	          uname -s > ${TESTVENVDIR}/.built; \
          fi))

######################################################################
## License and security

show-licenses: venv							## Show licenses of imported modules
	@ (cd ${PACKAGE_DIR} && source ${VENVDIR}/bin/activate && \
       python -m pip install --upgrade --disable-pip-version-check pip-licenses && \
       (cd ${PACKAGE_DIR} && pip-licenses 2>&1 | tee ${LICENSE_OUT}))

bandit-test: venv-test						## Run security test on source
	$(Q) echo "Running python security check with bandit on module code"
	@ (cd ${PACKAGE_DIR} && source ${TESTVENVDIR}/bin/activate && \
       python -m pip install --upgrade --disable-pip-version-check bandit && \
        (cd ${PACKAGE_DIR} && bandit -n 3 -r $(PACKAGE_DIR) -o bandit.log))

bandit-test-all: venv bandit-test			## Run security test on source and imported modules
	$(Q) echo "Running python security check with bandit on imports"
	@ (cd ${PACKAGE_DIR} && source ${TESTVENVDIR}/bin/activate && \
       python -m pip install --upgrade --disable-pip-version-check bandit && \
        (cd ${PACKAGE_DIR} && bandit -n 3 -r $(WORKING_DIR)/${VENVDIR} -o bandit.log))

######################################################################
## Testing

test: venv-test		## Run tox-based unit tests
	$(Q) echo "Executing unit tests w/tox"
	@ python -m pip install --upgrade --disable-pip-version-check tox && \
	   . ${TESTVENVDIR}/bin/activate && tox

frc-test: venv-test		## Run FRC unit tests
	$(Q) echo "Executing unit tests w/tox"
	@ python -m pip install --upgrade --disable-pip-version-check tox && \
	   . ${TESTVENVDIR}/bin/activate && tox

######################################################################
## Linting

lint: venv-test     ## Run lint on PON Automation using pylint
	@ (cd ${PACKAGE_DIR} && source ${TESTVENVDIR}/bin/activate && \
       pylint ${PYLINT_OPTS} ${PACKAGE_DIR} 2>&1 | tee ${PYLINT_OUT} && \
       echo; echo "See \"file://${PYLINT_OUT}\" for lint report")

########################################################
# Release related (Lint ran last since it probably will have errors until
# the code is refactored (which is not planned at this time)
## Release Procedures
release-check: distclean venv venv-test test bandit-test-all lint	## Clean distribution and run unit-test, security, and lint

######################################################################
## Utility
clean:		## Cleanup directory of build and test artifacts
	@ -rm -rf .tox *.coverage *.egg-info test/.pytest_cache ${PYLAMA_OUT} ${PYLINT_OUT} ${LICENSE_OUT}
	@ -rm -rf ${PACKAGE_DIR}/ctre_sim ${PACKAGE_DIR}/logs
	@ -find . -name '*.pyc' | xargs rm -f
	@ -find . -name '__pycache__' | xargs rm -rf
	@ -find . -name 'htmlcov' | xargs rm -rf
	@ -find . -name 'junit-report.xml' | xargs rm -rf
	@ -find . -name 'coverage.xml' | xargs rm -rf
	@ -find . -name '*.log' | xargs rm -rf
	@ -find . -name '*.wpilog' | xargs rm -rf
	@ -find . -name '._.DS_Store' | xargs rm -rf
	@ -find . -name 'ctre_sim' | xargs rm -rf

distclean: clean	## Cleanup all build, test, and virtual environment artifacts
	@ -rm -rf ${VENVDIR} ${TESTVENVDIR} ./vendordeps
	@ -find . -name 'simgui*.json' | xargs rm -rf
	@ -find . -name 'networktables.json' | xargs rm -rf

help: ## Print help for each Makefile target
	@echo ''
	@echo 'Usage:'
	@echo '  ${YELLOW}make${RESET} ${GREEN}<target> [<target> ...]${RESET}'
	@echo ''
	@echo 'Targets:'
	@awk 'BEGIN {FS = ":.*?## "} { \
		if (/^[a-zA-Z_-]+:.*?##.*$$/) {printf "    ${YELLOW}%-23s${GREEN}%s${RESET}\n", $$1, $$2} \
		else if (/^## .*$$/) {printf "  ${CYAN}%s${RESET}\n", substr($$1,4)} \
		}' $(MAKEFILE_LIST)

# end file
