# 2023 Robot Code for Team Linebreak

## Purpose

Code contained in this repository was made to control our robot for the 2023 FRC competition.

## Development

### Code Style

This project uses [Checkstyle](https://checkstyle.org) for code style enforcement and linting along with [Google's Java Style Guide](https://google.github.io/styleguide/javaguide.html).

#### VSCode Setup

1. Download the checkstyle plugin by [ShengChen](https://marketplace.visualstudio.com/items?itemName=shengchen.vscode-checkstyle)
2. Set the configuration file to `${workspaceFolder}/checkstyle.xml`
3. Set the version to `10.6.0`
4. Write code

Instructions on how to report errors and warnings via a CLI are in the works.

#### Modifications From Google's Java Style Guide

+ Indentations use 4 spaces instead of Google's 2
+ Line limit is 300 chars instead of Google's 100

## Licenses

Code written by the Team Linebreak Programming Team and other contributors is released to the public under the MIT license.

```txt
Copyright (c) 2023 Team Linebreak Programming Team and other contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### Software Dependencies

Gradle is licensed under [the Apache License v2](./gradle/LICENSE).

PMD is licensed under [a BSD-style license](./pmd/LICENSE).

WPILib is licensed under [their own license](./WPILib-License.md). Per the license terms, FIRST and WPILib **do not** endorse or promote this project or the software contained herein.
