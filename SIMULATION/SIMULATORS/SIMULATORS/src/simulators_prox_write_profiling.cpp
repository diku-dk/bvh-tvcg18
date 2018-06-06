#include <simulators_prox.h>

#include <util_profiling.h>
#include <util_matlab_write_profiling.h>
#include <util_log.h>

#include <string>
#include <vector>
#include <fstream>   // needed for std::ofstream

namespace simulators
{

  bool ProxEngine::write_profiling(std::string const & filename)
  {
    util::Log        logging;

    std::string const newline = util::Log::newline();

    std::ofstream matlab;

    matlab.open(filename.c_str(),std::ios::out);

    if(! matlab.is_open())
    {
      logging << "ProxEngine::write_profiling(): error could not open file = " << filename.c_str() << util::Log::newline();

      return false;
    }

    matlab << "%close all;" << std::endl;
    matlab << "%clear all;" << std::endl;
    matlab << "%clc;"       << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << util::matlab_write_profiling()  << std::endl;
    matlab << std::endl;
    matlab << std::endl;

    matlab << std::endl;
    matlab << std::endl;
    matlab <<  "%red   = [0.7,0.1,0.1];"   << std::endl;
    matlab <<  "%green = [0.1,0.7,0.1];" << std::endl;
    matlab <<  "%blue  = [0.1,0.1,0.7];"  << std::endl;
    matlab <<  "%f_type = 'Times';"      << std::endl;
    matlab <<  "%f_size = 20;"           << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%frames = [1:length(Ekin)];" << std::endl;
    matlab << "%Emech = Epot + Ekin;" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%figure(100);" << std::endl;
    matlab << "%e1 = plot(frames, Ekin,'r-'); " << std::endl;
    matlab << "%hold on; " << std::endl;
    matlab << "%e2 = plot(frames, Epot, 'g-'); " << std::endl;
    matlab << "%e3 = plot(frames, Emech, 'b-'); " << std::endl;
    matlab << "%hold off; " << std::endl;
    matlab << "%title('Kinetic and potential energy of total system', 'FontSize', f_size, 'FontName', f_type); "  << std::endl;
    matlab << "%legend([e1,e2,e3], {'Kinetic', 'Potential','Mechanical'}, 'FontSize', f_size, 'FontName', f_type); "  << std::endl;
    matlab << "%ylabel('Energy (Joules)', 'FontSize', f_size, 'FontName', f_type)" << std::endl;
    matlab << "%xlabel('Frame', 'FontSize', f_size, 'FontName', f_type)" << std::endl;
    matlab << "%print(gcf,'-depsc2','energy');" << std::endl;
    matlab << "%print(gcf,'-dpng','energy');" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%figure(200);" << std::endl;
    matlab << "%plot(abs(average_penetration)*100);" << std::endl;
    matlab << "%title('Penetrations', 'FontSize', f_size, 'FontName', f_type); " << std::endl;
    matlab << "%xlabel('Frame', 'FontSize', f_size, 'FontName', f_type); " << std::endl;
    matlab << "%ylabel('Penetration depth (cm)', 'FontSize', f_size, 'FontName', f_type)" << std::endl;
    matlab << "%print(gcf,'-depsc2','penetration');" << std::endl;
    matlab << "%print(gcf,'-dpng','penetration');" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%figure(300);" << std::endl;
    matlab << "%plot(contacts);" << std::endl;
    matlab << "%title('Contacts', 'FontSize', f_size, 'FontName', f_type); " << std::endl;
    matlab << "%xlabel('Frame', 'FontSize', f_size, 'FontName', f_type); " << std::endl;
    matlab << "%ylabel('Number of contacts', 'FontSize', f_size, 'FontName', f_type)" << std::endl;
    matlab << "%print(gcf,'-depsc2','contacts');" << std::endl;
    matlab << "%print(gcf,'-dpng','contacts');" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%figure(400);" << std::endl;
    matlab << "%plot(contacts(contacts>0) ,solver, '.'); "<< std::endl;
    matlab << "%title('Time used by solver vs number of contacts', 'FontSize', f_size, 'FontName', f_type); "<< std::endl;
    matlab << "%xlabel('Number of contacts', 'FontSize', f_size, 'FontName', f_type); "<< std::endl;
    matlab << "%ylabel('Solver (ms)', 'FontSize', f_size, 'FontName', f_type)" << std::endl;
    matlab << "%print(gcf,'-depsc2','solver_time_vs_contacts');" << std::endl;
    matlab << "%print(gcf,'-dpng','solver_time_vs_contacts');" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%figure(500);" << std::endl;
    matlab << "%plot(frames(contacts>0), solver); " << std::endl;
    matlab << "%title('Time used by solver per frame', 'FontSize', f_size, 'FontName', f_type)" << std::endl;
    matlab << "%xlabel('Frame', 'FontSize', f_size, 'FontName', f_type); " << std::endl;
    matlab << "%ylabel('Solver (ms)', 'FontSize', f_size, 'FontName', f_type)" << std::endl;
    matlab << "%print(gcf,'-depsc2','solver_time_per_frame');" << std::endl;
    matlab << "%print(gcf,'-dpng','solver_time_per_frame');" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%figure(600);" << std::endl;
    matlab << "%semilogy(cell2mat(convergence(1)));" << std::endl;
    matlab << "%hold on" << std::endl;
    matlab << "%for frame = 2:numel(convergence); " << std::endl;
    matlab << "%  semilogy(cell2mat(convergence(frame)));" << std::endl;
    matlab << "%end" << std::endl;
    matlab << "%title('Convergence', 'FontSize', f_size, 'FontName',f_type );" << std::endl;
    matlab << "%xlabel('Solver iteration', 'FontSize', f_size, 'FontName', f_type); " << std::endl;
    matlab << "%ylabel('Natural merit function', 'FontSize', f_size, 'FontName', f_type);" << std::endl;
    matlab << "%print(gcf,'-depsc2','convergence');" << std::endl;
    matlab << "%print(gcf,'-dpng','convergence');" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab << "%figure(700);" << std::endl;
    matlab << "%plot(cell2mat(rfactor(1)));" << std::endl;
    matlab << "%hold on" << std::endl;
    matlab << "%for frame = 2:numel(rfactor);" << std::endl;
    matlab << "%  plot(cell2mat(rfactor(frame)));" << std::endl;
    matlab << "%end" << std::endl;
    matlab << "%title('R-factor development', 'FontSize', f_size, 'FontName',f_type );" << std::endl;
    matlab << "%xlabel('Solver iteration', 'FontSize', f_size, 'FontName', f_type); ";
    matlab << "%ylabel('R-factor', 'FontSize', f_size, 'FontName', f_type);";
    matlab << "%print(gcf,'-depsc2','rfactors');" << std::endl;
    matlab << "%print(gcf,'-dpng','rfactors');" << std::endl;
    matlab << std::endl;
    matlab << std::endl;
    matlab.flush();
    matlab.close();

    logging << "ProxEngine::write_profiling(): Done writing profile data..." << newline;

    return true;
  }

} // namespace simulators