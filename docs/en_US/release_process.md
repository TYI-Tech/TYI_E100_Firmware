# Release Process

Use the following process when publishing a new `TYI E100` firmware release.

## Recommended order

1. finish code, configuration, and documentation updates
2. update [VERSION](../../VERSION)
3. update [CHANGELOG.md](../../CHANGELOG.md)
4. update [Release Notes](release_notes.md)
5. run the release check:

```bash
bash ./scripts/release_check.sh
```

6. push the branch to the remote repository
7. create a Git tag, for example:

```bash
git tag v0.1.1
git push origin v0.1.1
```

8. publish the version on the GitHub Release page

## Recommended GitHub Release content

- version number
- major changes
- build or deployment impact
- validated on-board test scope
- whether `machine.env` requires user changes

## Current repository guidance

- for runtime changes, update `CHANGELOG.md` and `release_notes.md` first
- for deployment-path changes, update `quick_start.md`, `operations.md`, and `faq.md`
- run `bash ./scripts/release_check.sh` before each release
