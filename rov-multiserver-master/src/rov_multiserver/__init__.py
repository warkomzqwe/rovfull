# -*- coding: utf-8 -*-
"""rov_multiserver.

ROV system control and video server.
"""

from pkg_resources import get_distribution, DistributionNotFound

try:  # pragma: no cover
    # Change here if project is renamed and does not equal the package name
    dist_name = 'rov-multiserver'
    __version__ = get_distribution(dist_name).version
except DistributionNotFound:  # pragma: no cover
    __version__ = 'unknown'
finally:  # pragma: no cover
    del get_distribution, DistributionNotFound
