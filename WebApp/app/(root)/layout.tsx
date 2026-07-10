import Sidebar from "@/components/Sidebar";
import RightSidebar from "@/components/RightSidebar";
import DemoModal from "@/components/DemoModal";
import PageTransition from "@/components/PageTransition";

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <main className="flex min-h-screen flex-col overflow-x-hidden">
      <DemoModal />
      <Sidebar />
      <RightSidebar />
      <PageTransition>
        <div className="flex-1 w-full min-w-0 pt-14 md:pt-0 md:pl-52 md:pr-14">
          {children}
        </div>
      </PageTransition>
    </main>
  );
}
