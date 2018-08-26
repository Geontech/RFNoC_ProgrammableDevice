# By default, the RPM will install to the standard REDHAWK SDR root location (/var/redhawk/sdr)
# You can override this at install time using --prefix /new/sdr/root when invoking rpm (preferred method, if you must)
%{!?_sdrroot: %global _sdrroot /var/redhawk/sdr}
%define _prefix %{_sdrroot}
Prefix:         %{_prefix}

# Point install paths to locations within our target SDR root
%define _sysconfdir    %{_prefix}/etc
%define _localstatedir %{_prefix}/var
%define _mandir        %{_prefix}/man
%define _infodir       %{_prefix}/info

Name:           RFNoC_ProgrammableDevice
Version:        1.0.0
Release:        1%{?dist}
Summary:        ExecutableDevice %{name}

Group:          REDHAWK/ExecutableDevices
License:        None
Source0:        %{name}-%{version}.tar.gz
BuildRoot:      %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)

BuildRequires:  redhawk-devel >= 2.0
Requires:       redhawk >= 2.0

BuildRequires:  RFNoC_RH-devel
Requires:       RFNoC_RH

# Interface requirements
BuildRequires:  frontendInterfaces >= 2.2 bulkioInterfaces >= 2.0
Requires:       frontendInterfaces >= 2.2 bulkioInterfaces >= 2.0


%description
ExecutableDevice %{name}
 * Commit: __REVISION__
 * Source Date/Time: __DATETIME__


%prep
%setup -q


%build
# Implementation cpp_x86
pushd cpp_x86
./reconf
%define _bindir %{_prefix}/dev/devices/RFNoC_ProgrammableDevice/cpp_x86
%configure
make %{?_smp_mflags}
popd
# Implementation cpp_armv7l
pushd cpp_armv7l
./reconf
%define _bindir %{_prefix}/dev/devices/RFNoC_ProgrammableDevice/cpp_armv7l
%configure
make %{?_smp_mflags}
popd


%install
rm -rf $RPM_BUILD_ROOT
# Implementation cpp_x86
pushd cpp_x86
%define _bindir %{_prefix}/dev/devices/RFNoC_ProgrammableDevice/cpp_x86
make install DESTDIR=$RPM_BUILD_ROOT
popd
# Implementation cpp_armv7l
pushd cpp_armv7l
%define _bindir %{_prefix}/dev/devices/RFNoC_ProgrammableDevice/cpp_armv7l
make install DESTDIR=$RPM_BUILD_ROOT
popd


%clean
rm -rf $RPM_BUILD_ROOT


%files
%defattr(-,redhawk,redhawk,-)
%dir %{_sdrroot}/dev/devices/RFNoC_ProgrammableDevice
%{_prefix}/dev/devices/RFNoC_ProgrammableDevice/RFNoC_ProgrammableDevice.scd.xml
%{_prefix}/dev/devices/RFNoC_ProgrammableDevice/RFNoC_ProgrammableDevice.prf.xml
%{_prefix}/dev/devices/RFNoC_ProgrammableDevice/RFNoC_ProgrammableDevice.spd.xml
%{_prefix}/dev/devices/RFNoC_ProgrammableDevice/cpp_x86
%{_prefix}/dev/devices/RFNoC_ProgrammableDevice/cpp_armv7l

