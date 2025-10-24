# **LLM Code prompt guidance**

# **Coding preferences for C++ Projects**

## **Language & toolchain**

- Target C++23 (minimum C++20). Treat all warnings as errors and enable the strict warning sets for GCC/Clang (`-Wall -Wextra -Wpedantic -Wconversion -Werror`).
- clang 17+ is the reference compiler; confirm portability with GCC 13+ before release. For MSVC, enable `/permissive-` and `/W4`.
- Use CMake 3.26+ as the primary build system, prefer Ninja for reproducible builds, and configure toolchain files for cross-compilation targets.
- Build in US English: source, diagnostics, comments, and user-facing messages must use US terminology.

## **Project layout**

- Follow a modern CMake package layout: top-level `CMakeLists.txt`, reusable modules in `cmake/`, public headers under `include/<project_name>/`, implementation sources in `src/<project_name>/`, and executables in `apps/`.
- Provide version metadata via `include/<project_name>/version.hpp` and installable CMake package exports under `cmake/`.
- Mirror the source hierarchy inside `tests/` for unit and integration coverage (Catch2, GoogleTest, or doctest wired through CTest). Place reproducible examples and demos in `examples/`.
- Keep developer tooling in `scripts/` (formatters, analyzers, packaging) and design documentation in `docs/`.

## **Build & dependencies**

- Pin dependencies through `FetchContent`, CPM.cmake, or vcpkg manifest mode; record exact versions and hashes. Never vendor large tarballs directly into source control.
- Enforce `CMAKE_CXX_EXTENSIONS OFF`, `POSITION_INDEPENDENT_CODE ON`, link-time optimization in release, and minimum standard requirements through `target_compile_features`.
- Gate optional features behind CMake options with sensible defaults and ensure CI covers every option combination.
- Integrate static analysis (`clang-tidy`, `cppcheck`), IWYU, and sanitizers (ASan, UBSan, TSan) into CI. Treat analyzer findings as failures unless the suppression is documented.

## **Configuration & environment**

- Maintain `config/env_template.txt` listing every environment variable, purpose, expected format, default fallback, and whether CI sets it. Annotate secrets with `# set in CI only`.
- Never commit real secrets or access tokens to templates, fixtures, or source; placeholders only. Real tokens belong in local `.env` files or approved secret stores.
- Centralize configuration parsing in `src/<project_name>/configuration/`, converting raw strings into validated structs or strong types before exposing them to the rest of the code.
- Provide deterministic defaults for non-sensitive settings and terminate fast with actionable diagnostics when required secrets are absent.
- Cache configuration values in immutable objects; avoid repeated calls to `std::getenv` across the codebase.

## **Security**

- Adopt a “secure by design” mindset: document threat models per subsystem, minimize attack surface in public headers, and default to least privilege for both processes and network interactions.
- Enforce memory safety by relying on RAII, smart pointers, standard containers, and `std::span`/`std::string_view` instead of raw pointers. Eliminate manual `new`/`delete`; if unavoidable, encapsulate them in dedicated owning abstractions.
- Compile with hardening flags (`-fstack-protector-strong`, `-D_FORTIFY_SOURCE=3`, `-fPIE`, `-Wl,-z,relro,-z,now`) and enable control-flow and pointer checks where available (`-fsanitize=safe-stack`, hardware CF protection).
- Validate and sanitize all external input (files, network payloads, CLI args). Parse into typed representations, reject invalid states early, and log the decision without echoing the original payload.
- Protect secrets: prefer OS key stores or dedicated secret managers, scrub sensitive buffers before release, and ensure logging, crash dumps, and telemetry never include credentials or personally identifiable data.
- Use vetted cryptographic libraries (libsodium, Botan, OpenSSL). Do not implement bespoke crypto; configure primitives with modern defaults (e.g., AES-GCM, ChaCha20-Poly1305, Ed25519).
- Continuously scan dependencies (SBOM, `vcpkg x-update-baseline`, `npm audit` equivalents) and keep a signed chain of custody for third-party code.
- Exercise the code with fuzzers (libFuzzer, AFL++) for parsers and boundary-heavy components, and run sanitizers plus Valgrind/Memcheck/AddressSanitizer in CI and nightly jobs.

## **Code style & naming**

- Favor descriptive `snake_case` for variables and functions, `PascalCase` for types and concepts, and `SCREAMING_SNAKE_CASE` for constants. Short indices (`i`, `j`) are acceptable in tight scopes but prefer descriptive names elsewhere.
- Declare `const`, `constexpr`, and `consteval` to show immutability. Use `auto` when it improves readability or avoids duplication; otherwise, spell out the type explicitly.
- Keep translation units cohesive: group related helpers in anonymous namespaces or `detail` sub-namespaces, and avoid sprinkling free functions across unrelated files.

## **Types & abstractions**

- Model domain concepts with `enum class`, wrapper structs, and `using` aliases to avoid primitive obsession.
- Prefer value semantics; expose views (`std::span`, `std::string_view`) for observation without transfer of ownership. Document lifetimes whenever returning references or views.
- Constrain templates with concepts or `requires` clauses, provide clear error messages, and keep template definitions in headers with explicit instantiations where build times demand it.
- Use `std::optional`, `std::variant`, and `std::expected` (or `tl::expected`) for nullable and fallible APIs; never rely on sentinel values or shared `bool` flags.

## **Memory & ownership**

- Let constructors and factories create fully formed objects. Default or delete special members explicitly to communicate ownership semantics.
- Prefer `std::unique_ptr` and `std::shared_ptr` with clear ownership transfer. Use `std::weak_ptr` only to break cycles and document the expected lifetime.
- Replace raw pointer parameters with references, `gsl::not_null`, or spans when null is not acceptable. When null is meaningful, use nullable smart pointers or `std::optional`.

## **Error handling & logging**

- Reserve exceptions for exceptional, recoverable situations; ensure strong exception safety guarantees by rolling back partial state on failure.
- When exceptions are unsuitable (performance or hot paths), return `std::expected<SuccessType, ErrorType>` and propagate rich context. Avoid plain `bool` or integer error codes.
- Centralize logging (spdlog/Boost.Log) and configure sinks once at startup. Prefer structured messages with key/value pairs and use parameter placeholders instead of manual concatenation.
- Wrap low-level exceptions with domain-specific context before rethrowing. Always log actionable steps, but never include secrets, API tokens, or personal data.

## **Concurrency**

- Use standard concurrency primitives (`std::thread`, `std::jthread`, `std::mutex`, `std::shared_mutex`, `std::atomic`) and higher-level abstractions (`std::async`, executors) instead of platform APIs unless necessary.
- Design thread-safe types explicitly: document invariants, leverage RAII locks, and keep critical sections minimal. Consider lock-free structures only with thorough benchmarks and correctness proofs.
- Test multithreaded code with Thread Sanitizer, race detectors, and stress tests. Provide deterministic synchronization points in tests to avoid flakiness.

## **HTTP & external APIs**

- Encapsulate HTTP and protocol clients in dedicated modules (libcurl, Boost.Beast, cpp-httplib). Convert responses into typed result objects immediately.
- Validate request payloads and enforce schemas before transmission. Classify errors into retryable and permanent categories.
- Apply bounded exponential backoff with jitter for transient faults, and surface telemetry for latency, retries, and error classes without leaking payload details.
- Include references to official API documentation alongside integration points and document rate limits or throttling behavior.

## **Documentation**

- Preserve existing comments when still accurate, and expand them to capture intent rather than restating code. Keep all documentation in ASCII.
- Annotate public headers with Doxygen-compatible comments. For complex subsystems, add architecture notes in `docs/` covering design choices, trade-offs, and extension points.
- Provide onboarding and runbook material that explains build targets, test commands, and deployment steps.

## **Testing**

- Co-develop unit tests with production code; use CTest to orchestrate Catch2/GoogleTest/doctest suites. Enforce deterministic seeds for pseudo-random behavior.
- Cover error paths, boundary conditions, concurrency behavior, and integration with external services. Gate long-running or hardware-dependent tests with explicit CTest labels.
- Integrate fuzzing, property-based tests (rapidcheck), and regression suites into CI. Run tests with `ctest --output-on-failure` and fail the pipeline on any warning or sanitizer finding.

## **Output & change management**

- Provide complete, drop-in replacements for modified translation units or headers rather than raw diffs unless explicitly requested.
- Focus narratives on design intent, trade-offs, and verification steps. Avoid line-by-line recitations of obvious edits.
- Keep module-level side effects minimal; prefer explicit initialization sequences in application entry points.

## **Mermaid diagrams**

- Use HTML break tags (`<br/>`) for line wrapping; never rely on literal newlines (`\n`) inside Mermaid labels.
- Ensure diagrams reflect the current architecture, including new layers introduced during development.
- Prefer concise node names and keep arrow annotations short to preserve readability in the VS Code plugin.

## **Versioning**

- Bump semantic versions whenever observable behavior, public APIs, or configuration inputs change. Update the version in `CMakeLists.txt`, `include/<project_name>/version.hpp`, and `README.md`.
- Maintain `CHANGELOG.md` in Keep a Changelog format, capturing enhancements, fixes, breaking changes, and migration notes.

## **Commits**

- Prefix commit messages with the new version tag (e.g., `v1.3.0 introduce swarm diagnostics.`) and keep messages imperative, scoped to a single logical change.
- Avoid mixing refactors, features, and formatting in one commit. Reference issue IDs when closing tickets or implementing tracked work.

## **Branch naming**

- Prefix branches with tracking IDs when available (e.g., `DRONE-42-swarm-pathing`). Otherwise, choose concise, kebab-case names that convey purpose.
