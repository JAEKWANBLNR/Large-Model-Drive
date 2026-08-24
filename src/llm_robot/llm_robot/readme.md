# llm_robot

Contains allow-listed tool servers for a mobile base, multiple configured robot namespaces, and an ARX5-style Cartesian arm. The mobile servers clamp velocity and duration, schedule an automatic stop, and reject unknown tools or robot namespaces.

Use `ROBOT_PROFILE=mobile`, `multi`, or `arm` so the model receives only the matching tool schema.
