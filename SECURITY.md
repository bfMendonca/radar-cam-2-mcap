# Security policy

If you find a security issue, please report it privately through
[GitHub security advisories](https://github.com/bfMendonca/radar-cam-2-mcap/security/advisories/new)
rather than opening a public issue.

The converter reads session folders and writes only to the output directory you give it. Treat
session folders from untrusted sources as untrusted input (they are parsed as JSON, CSV and H.264).
