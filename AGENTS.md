# AI Build Notes

## Compile

- Build this project from the repository root with `bake`.
- Example:

```sh
bake
```

## Run

- Run this project from the repository root with `bake run`.
- Example:

```sh
bake run
```

## Output

- The debug binary is written to `bin/x64-Linux-debug/example_gsolver`.

## Notes

- The project is configured through `project.json`.
- Do not invoke `gcc` directly unless the user explicitly asks for a manual build command; prefer `bake` for all normal compile steps.