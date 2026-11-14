################################################################################
# Passerelle Wi-Fi 6E des PAMI d'attaque (Ezurio Veda IF913, Infineon CYW55913)
#
# Projet ModusToolbox (Infineon), repris des réglages de l'exemple officiel
# mtb-example-threadx-wifi-udp-server. Voir README.md pour compiler et flasher.
################################################################################

MTB_TYPE=COMBINED

# Kit de test. Pour le kit Ezurio, mettre le BSP fourni par Ezurio s'il en
# existe un (sinon celui d'Infineon, même puce).
TARGET=CYW955913EVK-01

APPNAME=pami-bridge-veda-if913
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

COMPONENTS=
DISABLE_COMPONENTS=

SOURCES=
INCLUDES=

DEFINES=CY_RTOS_AWARE CY_RETARGET_IO_CONVERT_LF_TO_CRLF NX_SECURE_INCLUDE_USER_DEFINE_FILE NX_INCLUDE_USER_DEFINE_FILE NX_SECURE_ALLOW_SELF_SIGNED_CERTIFICATES
# Pays pour la réglementation radio (canaux 6 GHz autorisés en Europe)
DEFINES+= CY_WIFI_COUNTRY=WHD_COUNTRY_FRANCE
DEFINES += TX_PACKET_POOL_SIZE=10
DEFINES += RX_PACKET_POOL_SIZE=20
DEFINES += WCM_WORKER_THREAD_STACK_SIZE=5120
DEFINES += SECURE_SOCKETS_THREAD_STACKSIZE=1024

HEAP_SIZE=0x10000

VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=

################################################################################
# Chemins (ModusToolbox)
################################################################################

CY_APP_PATH=
CY_GETLIBS_SHARED_PATH=../
CY_GETLIBS_SHARED_NAME=mtb_shared
CY_COMPILER_PATH=

CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)
CY_TOOLS_PATHS+=
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

include $(CY_TOOLS_DIR)/make/start.mk
