#!/usr/bin/env python3
"""Write a Mid-360 config with your own host/LiDAR IPs.

The driver's config has to agree with reality in two places:
  * host_net_info.*.ip  -- the IP of the PC's NIC on the LiDAR subnet
  * lidar_configs[0].ip -- the LiDAR's own IP

The package ships the vendor example (host 192.168.1.50, LiDAR 192.168.1.160);
the Mid-360 usually arrives at 192.168.1.1xx, where xx comes from the last byte
of the serial number printed on its back. If you do not know it, run the driver
with the example config and look for "Lidar ... broadcast code" / the detected
IP in its log, or use Livox Viewer 2.

The output goes to a writable file (default ~/.config/handheld_mapping/), not
into the installed package share, so re-running colcon does not undo it.

Usage:
  ros2 run handheld_mapping set_mid360_ips.py                    # show local IPs
  ros2 run handheld_mapping set_mid360_ips.py --host-ip 192.168.1.50 --lidar-ip 192.168.1.160
  ... then pass it to the launch files:
  ros2 launch handheld_mapping mapping_livo2.py mid360_config:=<printed path>
"""
import argparse
import json
import os
import subprocess
import sys


def default_template():
    """The package's copy, whether run from source or from the install tree."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('handheld_mapping'), 'config', 'MID360_config.json'))
    except Exception:
        pass
    here = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.join(os.path.dirname(here), 'config', 'MID360_config.json'))
    candidates.append(os.path.join(here, '..', 'config', 'MID360_config.json'))
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[-1]


def local_ipv4():
    out = subprocess.run(['ip', '-4', '-o', 'addr', 'show'], capture_output=True, text=True)
    rows = []
    for line in out.stdout.splitlines():
        parts = line.split()
        if len(parts) > 3 and parts[2] == 'inet':
            rows.append((parts[1], parts[3]))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host-ip', help='IP of this PC on the LiDAR subnet')
    parser.add_argument('--lidar-ip', help="the Mid-360's IP")
    parser.add_argument('--template', default=None,
                        help='source config (default: the package copy)')
    parser.add_argument('--out', default=os.path.expanduser(
        '~/.config/handheld_mapping/MID360_config.json'))
    args = parser.parse_args()

    if not args.host_ip or not args.lidar_ip:
        print('local IPv4 addresses:')
        for iface, addr in local_ipv4():
            print(f'  {iface:<16} {addr}')
        print('\npass --host-ip and --lidar-ip to write a config')
        return 0 if not (args.host_ip or args.lidar_ip) else 2

    template = args.template or default_template()
    with open(template) as fh:
        cfg = json.load(fh)

    host = cfg['MID360']['host_net_info']
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        host[key] = args.host_ip
    cfg['lidar_configs'][0]['ip'] = args.lidar_ip

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as fh:
        json.dump(cfg, fh, indent=2)
        fh.write('\n')

    print(f'wrote {args.out}  (template: {template})')
    print(f'  host  {args.host_ip}   lidar {args.lidar_ip}')
    print('use it with:')
    print(f'  ros2 launch handheld_mapping mapping_livo2.py mid360_config:={args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
