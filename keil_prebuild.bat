@echo off
chcp 65001 >nul
REM Keil5 pre-build script - Auto generate version number
REM This script will be executed before Keil5 compilation

REM Set working directory to project root
cd /d "%~dp0"

REM Get current date in YYMMDD format using PowerShell (compatible with modern Windows)
for /f "delims=" %%a in ('powershell -command "Get-Date -Format 'yyMMdd'"') do set "CURRENT_DATE=%%a"

REM Check if Git is available
git --version >nul 2>&1
if errorlevel 1 (
    echo [Version Generation] Warning: Git not found, using default version
    goto :generate_default
)

REM Get Git information
for /f "delims=" %%i in ('git rev-parse --short HEAD 2^>nul') do set GIT_COMMIT_SHORT=%%i
if "%GIT_COMMIT_SHORT%"=="" set GIT_COMMIT_SHORT=unknown

for /f "delims=" %%i in ('git rev-parse HEAD 2^>nul') do set GIT_COMMIT_HASH=%%i
if "%GIT_COMMIT_HASH%"=="" set GIT_COMMIT_HASH=unknown

REM Check for uncommitted changes
set GIT_DIRTY=
for /f "delims=" %%i in ('git status --porcelain 2^>nul') do (
    set GIT_DIRTY=_dirty
    goto :dirty_check_done
)
:dirty_check_done

echo [Version Generation] Current Date: %CURRENT_DATE%
echo [Version Generation] Git Commit: %GIT_COMMIT_SHORT%
echo [Version Generation] Has Uncommitted Changes: %GIT_DIRTY%

:generate_version_h
REM Generate version.h file
(
echo /**
echo  *****************************************************************************
echo  * @file    version.h
echo  * @brief   Software version definition with Git commit information
echo  * @author  Gordon Li
echo  * @date    2025-01-XX
echo  * @version Auto-generated
echo  *****************************************************************************
echo  */
echo.
echo #ifndef VERSION_H
echo #define VERSION_H
echo.
echo /* Base version number with current date */
echo #define VERSION_BASE "TG%CURRENT_DATE%"
echo.
echo /* Git commit information - these values will be automatically replaced during build */
echo #define GIT_COMMIT_HASH "%GIT_COMMIT_HASH%"
echo #define GIT_COMMIT_SHORT "%GIT_COMMIT_SHORT%"
echo #define GIT_DIRTY "%GIT_DIRTY%"
echo.
echo /* Complete version format: base version + git short commit + dirty flag */
echo #define TGDMC VERSION_BASE "_" GIT_COMMIT_SHORT GIT_DIRTY
echo.
echo /* Version length limit */
echo #define VERSION_MAX_LENGTH 32
echo.
echo #endif /* VERSION_H */
) > version.h

echo [Version Generation] Version updated: TG%CURRENT_DATE%_%GIT_COMMIT_SHORT%%GIT_DIRTY%
echo [Version Generation] Complete!
exit /b 0

:generate_default
REM Generate default version (when Git is not available)
(
echo /**
echo  *****************************************************************************
echo  * @file    version.h
echo  * @brief   Software version definition with Git commit information
echo  * @author  Gordon Li
echo  * @date    2025-01-XX
echo  * @version Auto-generated
echo  *****************************************************************************
echo  */
echo.
echo #ifndef VERSION_H
echo #define VERSION_H
echo.
echo /* Base version number with current date */
echo #define VERSION_BASE "TG%CURRENT_DATE%"
echo.
echo /* Git commit information - these values will be automatically replaced during build */
echo #define GIT_COMMIT_HASH "unknown"
echo #define GIT_COMMIT_SHORT "unknown"
echo #define GIT_DIRTY ""
echo.
echo /* Complete version format: base version + git short commit + dirty flag */
echo #define TGDMC VERSION_BASE "_" GIT_COMMIT_SHORT GIT_DIRTY
echo.
echo /* Version length limit */
echo #define VERSION_MAX_LENGTH 32
echo.
echo #endif /* VERSION_H */
) > version.h

echo [Version Generation] Using default version: TG%CURRENT_DATE%_unknown
echo [Version Generation] Complete!
exit /b 0
