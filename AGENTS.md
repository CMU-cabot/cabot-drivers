# Repository Notes

- Do not build on the host machine. Host-side builds are not supported in this repository.
- When a build is needed, use `./build-workspace.sh`.
- Do not run `colcon build` directly on the host.
- Run unit tests in Docker:
  - all packages: `./unittest.sh`
  - single package: `docker compose run --rm driver ./script/unittest.sh -p <package> -b`
- When creating commits in this repository, use `git commit -s` to add a Signed-off-by line.
