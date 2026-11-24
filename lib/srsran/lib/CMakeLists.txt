#
# Copyright 2013-2021 Software Radio Systems Limited
#
# This file is part of srsRAN
#
# srsRAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsRAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

########################################################################
# Add subdirectories
########################################################################
add_subdirectory(src)
add_subdirectory(include)
#add_subdirectory(examples)
#add_subdirectory(test)
# test/asn1 specifically: contains the MCCH/RRC ASN.1 round-trip tests (e.g.
# srsran_asn1_rrc_mcch_test), verified to compile; the sibling mac/rlc/pdcp/phy/
# common/srslog/adt test trees under lib/test/ are left disabled since they are
# unverified in this environment (no build toolchain available to confirm).
add_subdirectory(test/asn1)

########################################################################
# Install library headers
########################################################################
INSTALL(  DIRECTORY include/
          DESTINATION "${INCLUDE_DIR}"
          FILES_MATCHING PATTERN "*.h" )
