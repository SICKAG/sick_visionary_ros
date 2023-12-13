# Contributing to SICK Visionary ROS

:+1::tada: Thank you for investing your time in contributing to our project! :tada::+1:

<div style="text-align: center;">
  <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f1/Logo_SICK_AG_2009.svg/1200px-Logo_SICK_AG_2009.svg.png" width="420">
</div>

## Code of Conduct

This project and everyone participating in it is governed by the following [Code of Conduct](CODE_OF_CONDUCT.md).
By participating, you are expected to uphold this code.

## If you have a question or are in need of technical support

Depending on the nature of your question, there are two support channels:

1. For questions regarding the code shared in this repo please check the [FAQ](#faq) first and [search if an issue already exists](../../issues).
   If a related issue doesn't exist, you can open a new issue using the [bug/issue form](../../issues/new/choose).
2. For application or device specific questions look for common solutions and knowledge articles on the [Sick Support Portal](https://support.sick.com/).
   If your question is not answered there, open a ticket on the [Sick Support Portal](https://support.sick.com/).

## How to contribute

> **IMPORTANT**: [**Contributions are subject to the Github Terms of service**](https://docs.github.com/en/site-policy/github-terms/github-terms-of-service#6-contributions-under-repository-license)
> "Whenever you add Content to a repository containing notice of a license, you license that Content under the same terms, and you agree that you have the right to license that Content under those terms. If you have a separate agreement to license that Content under different terms, such as a contributor license agreement, that agreement will supersede."

### Issues

#### Create a new issue

If you spot a problem, check the [FAQ](#faq) first and [search if an issue already exists](../../issues).
If a related issue doesn't exist, you can open a new issue using the [bug/issue form](../../issues/new/choose).

> **Note:** If you find a **Closed** issue that seems like it is the same thing that you're experiencing, open a new issue and include a link to the original issue in the body of your new one.

A good issue form includes:
- Description: Clear description of the issue.

- Steps to Reproduce
    1. First Step
    2. Second Step
    3. and so on…

- Expected behavior: What you expect to happen?

- Actual behavior: What actually happens?

- Reproduces how often: What percentage of the time does it reproduce?

- Versions: Which version of the software are you using? Either include the version tag or the commit hash.

- Additional Information: Any additional information, configuration or data that might be necessary to reproduce the issue.

#### Solve an issue

Scan through our [existing issues](../../issues) to find one that interests you.
You can narrow down the search using `labels` as filters.
See [Labels](##Issue-and-Pull-Request-Labels) for more information.
If you find an issue to work on, you are welcome to open a PR with a fix.

#### Make changes

Before making any changes make sure to follow our [Styleguide](#styleguide).
In general, we follow the "fork-and-pull" Git workflow.

 1. [**Fork**](https://docs.github.com/de/get-started/quickstart/fork-a-repo) the repo on GitHub
 2. [**Clone**](https://docs.github.com/de/repositories/creating-and-managing-repositories/cloning-a-repository) the project to your own machine
 3. Create a new feature **Branch**
 4. **Commit** changes to your own branch
 5. **Push** your work back up to your fork
 6. Submit a [**Pull request**](https://docs.github.com/de/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) so that we can review your changes

> **NOTE** Be sure to merge the latest from "upstream" before making a pull request!

### Enhacement
Enhancement suggestions are tracked as GitHub issues.

This section guides you through submitting an enhancement suggestion, including completely new features and minor improvements to existing functionality.
Following these guidelines helps maintainers and the community understand your suggestion and find related suggestions.

When you are creating an enhancement suggestion, please include as many details as possible.
Fill in the [feature template](../../issues/new/choose), including the steps that you imagine you would take if the feature you're requesting existed.

A good enhancement suggestion includes:
- Summary: One paragraph explanation of the feature.

- Motivation: Why are we doing this? What use cases does it support? What is the expected outcome?

- Describe alternatives: A clear and concise description of the alternative solutions you've considered.

- Additional context: Add any other context or screenshots about the feature request here.

## Styleguide

### Linting
Our C++ Code is linted with `clang-tidy`. Linting helps you analyse your code for potential errors. Clang-tidy will automatically detect the [.clang-tidy](.clang-tidy) file to analyse your code.
Follow these steps to use clang-tidy:
```bash
$ sudo apt-get install clang-tidy # install the clang-tidy package
$ clang-tidy -i <FILEDIR>/*.cpp -- # e.g. to lint all cpp files
```

### Formatting

We use `clang-format` so that our code is structured uniformly.
Clang-format will automatically detect the [.clang-format](.clang-format) file to format your code.
This will ensure that your code complies with our style guide.

Follow these steps to apply clang-formatting:
```bash
$ sudo apt-get install clang-format # install the clang-format package
$ clang-format -i <FILEDIR>/*.cpp # e.g. to format all cpp files
```
>**Note**
> Replace FILEDIR with the actual folderpath

Even better just use pre-commit in your clone, which will use the provided config to enforce the formatting even before committing locally.

## Issue and Pull Request Labels

## FAQ
