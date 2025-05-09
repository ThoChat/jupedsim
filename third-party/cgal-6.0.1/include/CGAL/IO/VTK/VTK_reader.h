// Copyright (c) 2018
// GeometryFactory (France) All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Stream_support/include/CGAL/IO/VTK/VTK_reader.h $
// $Id: include/CGAL/IO/VTK/VTK_reader.h 50cfbde3b84 $
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Stephane Tayeb

#ifndef CGAL_IO_VTK_READER_H
#define CGAL_IO_VTK_READER_H

#ifdef CGAL_USE_VTK

#include <vtkSmartPointer.h>
#include <vtkCommand.h>

#include <string>

namespace CGAL {
namespace IO {
namespace internal {

class ErrorObserverVtk
  : public vtkCommand
{
public:
  ErrorObserverVtk()
    : Error(false), Warning(false), ErrorMessage(""), WarningMessage("")
  { }

  static ErrorObserverVtk *New() { return new ErrorObserverVtk; }

  bool GetError() const { return this->Error; }
  bool GetWarning() const { return this->Warning; }
  std::string GetErrorMessage() { return ErrorMessage; }
  std::string GetWarningMessage() { return WarningMessage; }

  void Clear()
  {
    this->Error = false;
    this->Warning = false;
    this->ErrorMessage = "";
    this->WarningMessage = "";
  }

  virtual void Execute(vtkObject *vtkNotUsed(caller),
                       unsigned long event,
                       void *calldata)
  {
    switch (event)
    {
      case vtkCommand::ErrorEvent:
        ErrorMessage = static_cast<char *>(calldata);
        this->Error = true;
        break;
      case vtkCommand::WarningEvent:
        WarningMessage = static_cast<char *>(calldata);
        this->Warning = true;
        break;
    }
  }

private:
  bool Error;
  bool Warning;
  std::string ErrorMessage;
  std::string WarningMessage;
};

template <class vtkReader>
vtkSmartPointer<vtkReader> read_vtk_file(const std::string& fname,
                                         vtkSmartPointer<ErrorObserverVtk> errorObserver)
{
  vtkSmartPointer<vtkReader> reader = vtkSmartPointer<vtkReader>::New();
  reader->AddObserver(vtkCommand::ErrorEvent, errorObserver);
  reader->AddObserver(vtkCommand::WarningEvent, errorObserver);
  reader->SetFileName(fname.data());
  reader->Update();

  return reader;
}

} // namespace internal
} // namespace IO
} // namespace CGAL

#endif // CGAL_USE_VTK

#endif // CGAL_IO_VTK_READER_H
