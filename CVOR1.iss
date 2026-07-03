; Inno Setup Script for VOR / ASRACS / SAAF Monitoring System v5.2
; ============================================================================

#define MyAppName "VOR / ASRACS / SAAF Monitoring System"
#define MyAppVersion "5.2"
#define MyAppPublisher "South African Air Force (SAAF)"
#define MyAppURL "https://github.com/schalkpieterse76-afk/VOR-Monitoring--System"
#define MyAppExeName "CVOR1.exe"
#define MyAppCopyright "Copyright 2026 Aviation Systems. All rights reserved."

[Setup]
AppId={{3A7F8C2D-4E9B-4F5A-8B6C-9D2E1F4A5C6B}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}/issues
AppUpdatesURL={#MyAppURL}/releases
DefaultDirName={autopf}\{#StringChange(MyAppName, ' ', '')}
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
LicenseFile=LICENSE.txt
InfoBeforeFile=INSTALL.txt
InfoAfterFile=README.txt
OutputDir=Output
OutputBaseFilename=CVOR1_Setup_v5.2
SetupIconFile=CVOR1.ico
UninstallDisplayIcon={app}\{#MyAppExeName}
Compression=lzma
SolidCompression=yes
WizardStyle=modern
MinVersion=6.1sp1
ArchitecturesInstallIn64BitMode=x64
ArchitecturesAllowed=x64

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked
Name: "quicklaunchicon"; Description: "{cm:CreateQuickLaunchIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked; OnlyBelowVersion: 6.1; Check: not IsAdminInstallMode

[Files]
Source: "dist\CVOR1\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "dist\CVOR1\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "CVOR1.ico"; DestDir: "{app}"; Flags: ignoreversion
Source: "README.txt"; DestDir: "{app}"; Flags: ignoreversion
Source: "LICENSE.txt"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\CVOR1.ico"
Name: "{group}\{cm:UninstallProgram,{#MyAppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon; IconFilename: "{app}\CVOR1.ico"
Name: "{autoquicklaunch}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: quicklaunchicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

[UninstallDelete]
Type: filesandordirs; Name: "{app}\vor_monitor.log"
Type: filesandordirs; Name: "{app}\vor_config.yaml"
