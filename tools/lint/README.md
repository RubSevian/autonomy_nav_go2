# Offline ROS manifest validation

`catalog.xml` maps the ROS package format 3 schema URL to the local snapshot,
including `package_common.xsd`. This preserves schema validation in tests with
no network access. The XSD files are unmodified third-party material from
https://github.com/ros-infrastructure/rep/tree/master/xsd (retrieved 2026-11-08),
not relicensed under this project's MIT license.

SHA-256:

  f096a197ed6d7878984bb2501a55f7f1bd4895d254399fb7857e154bfb644f41  package_format3.xsd
  941ea8645344f3c4b7b9d7e68799898309d65a18225fa9cbef4169d95d1a3211  package_common.xsd
