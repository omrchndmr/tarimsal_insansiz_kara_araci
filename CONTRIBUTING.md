# Contributing

Contributions, bug reports, and suggestions are welcome.

## How to Contribute

1. Fork this repository.
2. Create a feature branch: `git checkout -b feature/your-feature-name`
3. Commit your changes with a clear message: `git commit -m "feat: describe your change"`
4. Push to your fork: `git push origin feature/your-feature-name`
5. Open a Pull Request against the `main` branch.

## Code Standards

- Python code should follow [PEP 8](https://peps.python.org/pep-0008/).
- Use `rospy.loginfo/logwarn/logerr/logdebug` instead of `print()`.
- Avoid hardcoded paths or magic numbers; use constants or ROS parameters.
- Keep ROS callbacks non-blocking (avoid `time.sleep()` in callbacks).

## Reporting Issues

Open a GitHub Issue with:
- A description of the problem
- Steps to reproduce
- ROS log output or error messages
- Hardware configuration if relevant
