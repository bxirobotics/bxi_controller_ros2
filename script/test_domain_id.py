#!/usr/bin/env python3
"""Check generated configs and entry-point preflight without changing the system."""

import os
from pathlib import Path
import subprocess

import pytest


SCRIPTS = Path(__file__).resolve().parent
CASES = [
    ("elf3-001", 31),
    ("elf3-arm-08.local", 38),
    ("elf3-124", 154),
    ("elf3-202", 232),
    ("elf3-203", 1),
    ("elf3-204", 2),
    ("elf3-232", 30),
    ("elf3-233", 31),
    ("elf3-255", 53),
    ("elf3-434", 232),
    ("elf3-435", 1),
    ("elf3-436", 2),
    ("elf3-999", 101),
]


@pytest.fixture
def config_env(tmp_path):
    hostname = tmp_path / "hostname"
    hostname.write_text('#!/bin/sh\nprintf "%s\\n" "$TEST_HOSTNAME"\n')
    hostname.chmod(0o755)
    return {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "BXI_ROBOT_CONFIG_FILE": str(tmp_path / "robot_config.yaml"),
    }


@pytest.mark.parametrize("hostname, domain", CASES)
def test_generated_configs_agree(config_env, hostname, domain):
    config_env["TEST_HOSTNAME"] = hostname
    subprocess.run(
        ["bash", str(SCRIPTS / "generate_robot_config.sh")],
        env=config_env, capture_output=True, text=True, check=True,
    )
    config = Path(config_env["BXI_ROBOT_CONFIG_FILE"]).read_text()
    assert f"ros_domain_id: {domain}" in config.splitlines()
    assert f"robot_id: '{hostname.split('.')[0].split('-')[-1]}'" in config.splitlines()
    service = subprocess.run(
        ["bash", str(SCRIPTS / "generate_ros_config.sh"),
         str(SCRIPTS / "ros_elf_launch.service"), str(domain)],
        capture_output=True, text=True, check=True,
    )
    assert f"Environment=ROS_DOMAIN_ID={domain}" in service.stdout.splitlines()


@pytest.mark.parametrize("script", ["deploy_autostart.sh", "usr_pro.sh", "test_pro.sh"])
@pytest.mark.parametrize("hostname, domain", CASES)
def test_entry_point_preflight(config_env, script, hostname, domain):
    config_env["TEST_HOSTNAME"] = hostname
    # Run only the hostname preflight block; never invoke sudo or deployment.
    source = (SCRIPTS / script).read_text()
    start = source.index('short_hostname="$(hostname)"')
    preflight = source[start:].split("\n\n", 1)[0]
    result = subprocess.run(
        ["bash", "-euc", preflight + '\nprintf "%s\\n" "$domain"'],
        env=config_env, capture_output=True, text=True, check=True,
    )
    assert result.stdout.strip() == str(domain)


@pytest.mark.parametrize("hostname", ["elf", "elf3-invalid", "elf3-1000", "bad host-203"])
def test_invalid_hostname_preserves_config(config_env, hostname):
    config_env["TEST_HOSTNAME"] = hostname
    output = Path(config_env["BXI_ROBOT_CONFIG_FILE"])
    output.write_text("existing config\n")
    result = subprocess.run(
        ["bash", str(SCRIPTS / "generate_robot_config.sh")],
        env=config_env, capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert output.read_text() == "existing config\n"


@pytest.mark.parametrize("domain", ["0", "000", "233", "255", "-1", "abc"])
def test_explicit_domain_is_validated(domain):
    result = subprocess.run(
        ["bash", str(SCRIPTS / "generate_ros_config.sh"),
         str(SCRIPTS / "ros_elf_launch.service"), domain],
        capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert not result.stdout
